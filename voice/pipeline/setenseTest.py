import nltk

def split_paragraph(paragraph):
    # Ensure NLTK sentence tokenizer is downloaded
    nltk.download('punkt', quiet=True)
    
    # Use NLTK's sent_tokenize function to split the paragraph
    sentences = nltk.sent_tokenize(paragraph)
    
    # Split sentences further if they exceed 100 characters
    final_sentences = []
    for sentence in sentences:
        if len(sentence) > 150:
            # Split by comma
            comma_splits = sentence.split(',')
            temp_sentence = ''
            for split in comma_splits:
                if len(temp_sentence) + len(split) < 150:
                    temp_sentence += split + ','
                else:
                    final_sentences.append(temp_sentence.strip(','))
                    temp_sentence = split + ','
            final_sentences.append(temp_sentence.strip(','))
        else:
            final_sentences.append(sentence)
    
    return final_sentences


import persistqueue
q = persistqueue.SQLiteQueue('audio', auto_commit=True)


# Example usage:
# paragraph = "Handles edge cases: nltk's tokenizer is trained on a wide range of texts. so it can handle various edge cases, such as abbreviations, titles, and other instances where periods do not indicate the end of a sentence.Supports multiple languages: nltk supports tokenization for multiple languages, so if your text contains sentences in languages other than English, nltk can still accurately tokenize them.More accurate: nltk's tokenizer uses more sophisticated rules and heuristics than simple period splitting, leading to more accurate sentence boundary detection. Easy to extend and customize: If you need to handle specific cases or improve tokenization for your domain, nltk allows you to customize and extend its tokenization rules."
# sentences = split_paragraph(paragraph)
# for s in sentences:
#     print(s)
#     # q.put(s)
#     # put to queue
#     print('\n')

# print(sentences)

# print(len("Handles edge cases: nltk's tokenizer is trained on a wide range of texts"))
    

def check_wake_words(sentence, wake_words=['hi robot', 'hey robot']):
    for word in wake_words:
        if word in sentence.lower():
            print(True)
            return
    print(False)

# Test the function
check_wake_words('Hi Robot, how are you?')
