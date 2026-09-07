#!/usr/bin/env python3

import argparse
import os
import pickle
import sys

import numpy as np
from insightface.app import FaceAnalysis

IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Enroll known faces into a face DB pickle for guide_perception.')
    parser.add_argument('--images-dir', required=True,
                        help='Directory containing one subfolder per person.')
    parser.add_argument('--out', required=True,
                        help='Output path for the {name: embedding} pickle file.')
    parser.add_argument('--ctx-id', type=int, default=-1,
                        help='InsightFace ctx_id (-1 = CPU, >=0 = GPU index). Default -1.')
    return parser.parse_args()


def _largest_face(faces):
    # Return the face with the largest bbox area, or None if faces is empty.
    if not faces:
        return None
    return max(faces, key=lambda f: max(0.0, f.bbox[2] - f.bbox[0]) * max(0.0, f.bbox[3] - f.bbox[1]))


def _list_person_dirs(images_dir):
    result = []
    for d in os.listdir(images_dir):
        if os.path.isdir(os.path.join(images_dir, d)):
            result.append(d)
    return sorted(result)


def _list_images(person_dir):
    result = []
    for f in os.listdir(person_dir):
        if f.lower().endswith(IMAGE_EXTENSIONS):
            result.append(f)
    return sorted(result)


def enroll(images_dir, ctx_id):
    import cv2

    face_app = FaceAnalysis(name='buffalo_l')
    face_app.prepare(ctx_id=ctx_id)

    person_dirs = _list_person_dirs(images_dir)
    if not person_dirs:
        print(f'No person subfolders found in {images_dir}.')
        return {}

    face_db = {}
    print(f'Found {len(person_dirs)} person folder(s) in {images_dir}.')

    for person in person_dirs:
        person_path = os.path.join(images_dir, person)
        image_files = _list_images(person_path)

        if not image_files:
            print(f'  [warn] {person}: no images found, skipping.')
            continue

        embeddings = []
        for image_file in image_files:
            image_path = os.path.join(person_path, image_file)
            frame = cv2.imread(image_path)
            if frame is None:
                print(
                    f'  [warn] {person}/{image_file}: failed to read image, skipping.')
                continue

            faces = face_app.get(frame)
            if not faces:
                print(
                    f'  [warn] {person}/{image_file}: no face detected, skipping.')
                continue

            if len(faces) > 1:
                print(
                    f'  [warn] {person}/{image_file}: multiple faces detected, using largest bbox.')

            face = _largest_face(faces)
            embedding = np.asarray(face.embedding, dtype=np.float64)
            norm = np.linalg.norm(embedding)
            if norm == 0:
                print(
                    f'  [warn] {person}/{image_file}: zero-norm embedding, skipping.')
                continue
            embeddings.append(embedding / norm)

        if not embeddings:
            print(
                f'  [warn] {person}: zero usable embeddings, skipping person.')
            continue

        mean_embedding = np.mean(embeddings, axis=0)
        mean_norm = np.linalg.norm(mean_embedding)
        if mean_norm == 0:
            print(
                f'  [warn] {person}: averaged embedding has zero norm, skipping person.')
            continue

        face_db[person] = mean_embedding / mean_norm
        print(
            f'  [ok] {person}: enrolled using {len(embeddings)}/{len(image_files)} image(s).')

    return face_db


def main():
    args = parse_args()

    images_dir = os.path.expanduser(args.images_dir)
    out_path = os.path.expanduser(args.out)

    if not os.path.isdir(images_dir):
        print(
            f'Error: images-dir {images_dir} does not exist or is not a directory.', file=sys.stderr)
        sys.exit(1)

    face_db = enroll(images_dir, args.ctx_id)

    if not face_db:
        print('No people were enrolled. Not writing output file.')
        sys.exit(1)

    out_dir = os.path.dirname(out_path)
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    with open(out_path, 'wb') as f:
        pickle.dump(face_db, f)

    print()
    print('Summary:')
    print(f'  People enrolled: {len(face_db)}')
    for name in sorted(face_db.keys()):
        print(f'    - {name}')
    print(f'  Saved face DB to: {out_path}')


if __name__ == '__main__':
    main()
