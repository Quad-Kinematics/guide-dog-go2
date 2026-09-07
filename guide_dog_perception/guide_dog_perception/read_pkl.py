import pickle

# Load the pickle file
with open('config/face_db.pkl', 'rb') as f:
    face_db = pickle.load(f)

# Print keys (names of enrolled people) and data types
print("Enrolled people:", list(face_db.keys()))
for name, embedding in face_db.items():
    print(f" - {name}: shape {embedding.shape}, dtype {embedding.dtype}")
