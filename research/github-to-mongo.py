"""
github_to_mongodb_qdrant.py

Purpose:
- Fetch GitHub repo metadata and README
- Extract structured fields (name, description, stars, language, topics, license, last_update)
- Extract feature list & categories (via OpenAI if API key provided; otherwise simple heuristic)
- Generate embeddings using sentence-transformers (open-source)
- Store structured data in MongoDB
- Store vectors in Qdrant (open-source vector DB)
- Provide a simple search function that queries Qdrant and returns full project info from MongoDB

Requirements (pip):
    pip install requests pymongo qdrant-client sentence-transformers openai

Services expected running/configured:
- MongoDB (local or remote) accessible via MONGO_URI
- Qdrant server running (local default http://localhost:6333) or QDRANT_URL

Environment variables (recommended):
- GITHUB_TOKEN (optional but recommended to increase rate limits)
- MONGO_URI (e.g. mongodb://localhost:27017)
- QDRANT_URL (e.g. http://localhost:6333)
- OPENAI_API_KEY (optional; if present will be used for higher-quality feature extraction/labeling)

Usage example:
    python github_to_mongodb_qdrant.py --repo "psf/requests"

"""

import os
import sys
import time
import json
import argparse
from typing import List, Dict, Any, Optional

import requests
from pymongo import MongoClient, ASCENDING
from qdrant_client import QdrantClient
from qdrant_client.http import models as rest_models
from sentence_transformers import SentenceTransformer

# Optional OpenAI for better extraction (if user provides key)
try:
    import openai
    HAVE_OPENAI = True
except Exception:
    HAVE_OPENAI = False

# ------------------------- Config -------------------------
GITHUB_API = "https://api.github.com"
GITHUB_TOKEN = os.getenv('GITHUB_TOKEN')
MONGO_URI = os.getenv('MONGO_URI', 'mongodb://localhost:27017')
MONGO_DB = 'github_catalog'
QDRANT_URL = os.getenv('QDRANT_URL', 'http://localhost:6333')
QDRANT_COLLECTION = 'repo_embeddings'
EMBED_MODEL_NAME = 'all-MiniLM-L6-v2'  # small & fast; change if you want bigger
EMBED_DIM = 384  # all-MiniLM-L6-v2 => 384 dims

# ------------------------- Helpers -------------------------

def github_headers():
    headers = {'Accept': 'application/vnd.github.v3+json'}
    if GITHUB_TOKEN:
        headers['Authorization'] = f'token {GITHUB_TOKEN}'
    return headers


def fetch_repo(repo_full_name: str) -> Dict[str, Any]:
    """Fetch repository metadata from GitHub API.
    repo_full_name: e.g. 'psf/requests'
    """
    url = f"{GITHUB_API}/repos/{repo_full_name}"
    r = requests.get(url, headers=github_headers())
    r.raise_for_status()
    repo = r.json()

    # README
    readme = ''
    try:
        rr = requests.get(f"{GITHUB_API}/repos/{repo_full_name}/readme", headers=github_headers())
        if rr.status_code == 200:
            rrj = rr.json()
            if 'content' in rrj and 'encoding' in rrj and rrj['encoding'] == 'base64':
                import base64
                readme = base64.b64decode(rrj['content']).decode(errors='replace')
            else:
                readme = rrj.get('content', '')
    except Exception:
        readme = ''

    topics = []
    try:
        tr = requests.get(f"{GITHUB_API}/repos/{repo_full_name}/topics", headers={**github_headers(), 'Accept': 'application/vnd.github.mercy-preview+json'})
        if tr.status_code == 200:
            topics = tr.json().get('names', [])
    except Exception:
        topics = []

    return {
        'full_name': repo.get('full_name'),
        'name': repo.get('name'),
        'owner': repo.get('owner', {}).get('login'),
        'description': repo.get('description'),
        'stars': repo.get('stargazers_count'),
        'forks': repo.get('forks_count'),
        'language': repo.get('language'),
        'license': repo.get('license', {}).get('name') if repo.get('license') else None,
        'repo_url': repo.get('html_url'),
        'updated_at': repo.get('updated_at'),
        'readme': readme,
        'topics': topics,
    }


# ------------------------- Feature extraction -------------------------

OPENAI_PROMPT = """
You are an assistant that extracts a concise JSON summary from a GitHub repository README/description.
Return JSON with keys: features (list of short strings), categories (list), suitable_scenarios (list), brief_summary (1-2 sentences).
Only output valid JSON.

Readme/Description:
"""


def extract_with_openai(text: str) -> Dict[str, Any]:
    if not HAVE_OPENAI:
        raise RuntimeError('openai library not installed')
    openai.api_key = os.getenv('OPENAI_API_KEY')
    if not openai.api_key:
        raise RuntimeError('OPENAI_API_KEY not set')

    prompt = OPENAI_PROMPT + text
    resp = openai.ChatCompletion.create(
        model='gpt-4o-mini',
        messages=[{'role':'user','content':prompt}],
        max_tokens=512,
        temperature=0.0,
    )
    content = resp['choices'][0]['message']['content']
    # Try to find JSON substring
    start = content.find('{')
    end = content.rfind('}')
    if start == -1 or end == -1:
        raise ValueError('OpenAI response did not contain JSON')
    json_text = content[start:end+1]
    return json.loads(json_text)


def heuristic_extract(text: str) -> Dict[str, Any]:
    """Fallback extractor: extract headings and bullet lists as features, use topics to form categories."""
    lines = text.splitlines()
    features = []
    categories = set()
    scenarios = []

    # collect lines that look like bullet points
    for ln in lines:
        s = ln.strip()
        if s.startswith('- ') or s.startswith('* ') or s.startswith('+ '):
            features.append(s[2:].strip())
        # headings
        if s.startswith('##'):
            h = s.lstrip('#').strip().lower()
            if 'feature' in h or 'capability' in h or 'what' in h:
                categories.add('feature-explained')
            if 'example' in h or 'use' in h:
                categories.add('usage')
    # take first paragraph as brief summary
    paras = [p.strip() for p in text.split('\n\n') if p.strip()]
    brief = paras[0][:400] if paras else ''

    # simple scenario extraction by keyword heuristics
    kws = {
        'robot': 'robotics', 'robotic': 'robotics', 'controller': 'robotics',
        'ml': 'machine-learning', 'model': 'machine-learning', 'inference': 'machine-learning',
        'web': 'web', 'server': 'backend', 'cli': 'cli',
    }
    for k,v in kws.items():
        if k in text.lower():
            categories.add(v)
    # fallback: top 3 sentences as features
    if not features:
        import re
        sents = re.split(r'(?<=[.!?])\s+', brief)
        features = sents[:3]

    return {
        'features': features,
        'categories': list(categories),
        'suitable_scenarios': scenarios,
        'brief_summary': brief
    }


def extract_structured_info(repo_meta: Dict[str, Any]) -> Dict[str, Any]:
    text = '\n\n'.join(filter(None, [repo_meta.get('description',''), repo_meta.get('readme','')]))
    extractor = None
    if HAVE_OPENAI and os.getenv('OPENAI_API_KEY'):
        try:
            extractor = extract_with_openai
        except Exception as e:
            print('OpenAI extraction failed, falling back to heuristic:', e)
            extractor = heuristic_extract
    else:
        extractor = heuristic_extract

    out = extractor(text)
    # normalize
    out['features'] = out.get('features') or []
    out['categories'] = out.get('categories') or repo_meta.get('topics', [])
    out['suitable_scenarios'] = out.get('suitable_scenarios') or []
    out['brief_summary'] = out.get('brief_summary') or repo_meta.get('description') or ''
    return out

# ------------------------- Embeddings & Vector DB -------------------------

EMBED_MODEL = None

def load_embed_model():
    global EMBED_MODEL
    if EMBED_MODEL is None:
        EMBED_MODEL = SentenceTransformer(EMBED_MODEL_NAME)
    return EMBED_MODEL


def embed_texts(texts: List[str]) -> List[List[float]]:
    model = load_embed_model()
    vectors = model.encode(texts, show_progress_bar=False, convert_to_numpy=True)
    return vectors.tolist()


def init_qdrant():
    # parse host/port from QDRANT_URL
    client = QdrantClient(url=QDRANT_URL)
    # create collection if not exists
    try:
        client.get_collection(collection_name=QDRANT_COLLECTION)
    except Exception:
        client.recreate_collection(
            collection_name=QDRANT_COLLECTION,
            vectors_config=rest_models.VectorParams(size=EMBED_DIM, distance=rest_models.Distance.COSINE),
        )
    return client

# ------------------------- MongoDB -------------------------

def init_mongo():
    client = MongoClient(MONGO_URI)
    db = client[MONGO_DB]
    # indexes
    db.projects.create_index([('full_name', ASCENDING)], unique=True)
    db.features.create_index([('project_id', ASCENDING)])
    return db

# ------------------------- Main ingestion flow -------------------------

def ingest_repo(repo_full_name: str, db, qclient: QdrantClient):
    print('Fetching', repo_full_name)
    repo = fetch_repo(repo_full_name)
    print('Extracting structured info...')
    extracted = extract_structured_info(repo)

    # Prepare project doc
    project_doc = {
        'full_name': repo['full_name'],
        'name': repo['name'],
        'owner': repo['owner'],
        'description': repo['description'],
        'stars': repo['stars'],
        'forks': repo['forks'],
        'language': repo['language'],
        'license': repo['license'],
        'repo_url': repo['repo_url'],
        'updated_at': repo['updated_at'],
        'topics': repo['topics'],
        'brief_summary': extracted.get('brief_summary'),
        'raw_readme_snippet': (repo['readme'] or '')[:2000],
        'ingested_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    }

    # upsert project
    res = db.projects.update_one({'full_name': project_doc['full_name']}, {'$set': project_doc}, upsert=True)
    project_id = db.projects.find_one({'full_name': project_doc['full_name']})['_id']

    # features
    db.features.delete_many({'project_id': project_id})
    features_to_insert = []
    for f in extracted.get('features', []):
        features_to_insert.append({'project_id': project_id, 'feature_text': f})
    if features_to_insert:
        db.features.insert_many(features_to_insert)

    # Prepare embeddings: we will create one document per meaningful chunk: title/summary + each feature
    docs = []
    chunks = []
    # main summary chunk
    chunks.append({'type': 'summary', 'text': project_doc['brief_summary'] or project_doc['description'] or ''})
    for i, f in enumerate(extracted.get('features', [])):
        chunks.append({'type': 'feature', 'text': f, 'feature_index': i})

    # Also add topics as a chunk
    if repo['topics']:
        chunks.append({'type': 'topics', 'text': ', '.join(repo['topics'])})

    texts = [c['text'] for c in chunks]
    vectors = embed_texts(texts)

    # Upsert into Qdrant
    points = []
    for i, (c, v) in enumerate(zip(chunks, vectors)):
        payload = {
            'project_full_name': repo['full_name'],
            'project_mongo_id': str(project_id),
            'chunk_type': c.get('type'),
            'text': c.get('text')[:2000]
        }
        points.append(rest_models.PointStruct(id=None, vector=v, payload=payload))

    # upload
    qclient.upsert(collection_name=QDRANT_COLLECTION, points=points)

    # store categories/scenarios into project doc as well
    db.projects.update_one({'_id': project_id}, {'$set': {'categories': extracted.get('categories', []), 'suitable_scenarios': extracted.get('suitable_scenarios', [])}})

    print('Ingested', repo_full_name)
    return project_id

# ------------------------- Simple search API -------------------------

def search_query(query: str, qclient: QdrantClient, db, top_k: int = 5):
    qvec = embed_texts([query])[0]
    hits = qclient.search(collection_name=QDRANT_COLLECTION, query_vector=qvec, limit=top_k)
    results = []
    for h in hits:
        payload = h.payload
        proj = db.projects.find_one({'full_name': payload['project_full_name']})
        results.append({'score': h.score, 'payload': payload, 'project': proj})
    return results

# ------------------------- CLI -------------------------

def main():
    parser = argparse.ArgumentParser(description='Ingest GitHub repos into MongoDB + Qdrant')
    parser.add_argument('--repo', type=str, help='single repo full name e.g. owner/repo')
    parser.add_argument('--file', type=str, help='file with list of repos, one per line')
    parser.add_argument('--search', type=str, help='run a test search after ingest')
    args = parser.parse_args()

    if HAVE_OPENAI and os.getenv('OPENAI_API_KEY'):
        openai.api_key = os.getenv('OPENAI_API_KEY')

    db = init_mongo()
    qclient = init_qdrant()

    repos = []
    if args.repo:
        repos = [args.repo]
    elif args.file:
        with open(args.file, 'r') as f:
            repos = [l.strip() for l in f if l.strip()]
    else:
        print('please pass --repo owner/repo or --file list.txt')
        sys.exit(1)

    for r in repos:
        try:
            ingest_repo(r, db, qclient)
        except Exception as e:
            print('failed to ingest', r, e)

    if args.search:
        print('\nSearch results for:', args.search)
        res = search_query(args.search, qclient, db, top_k=5)
        print(json.dumps(res, default=str, indent=2))

if __name__ == '__main__':
    main()
