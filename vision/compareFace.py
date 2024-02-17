import face_recognition

known_name = 'Yan'
known_image = face_recognition.load_image_file("face_1708150594.png")
unknown_image = face_recognition.load_image_file("face_1708150604.png")


known_encoding = face_recognition.face_encodings(known_image)[0]
unknown_encoding = face_recognition.face_encodings(unknown_image)[0]

results = face_recognition.compare_faces([known_encoding], unknown_encoding)
if results == [True]:
    print(known_name)
else:
    print('unknown name')

# [False]
# [True]
