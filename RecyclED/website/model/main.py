from keras.models import load_model  # TensorFlow is required for Keras to work
from flask import Flask, request
from PIL import Image, ImageOps
from io import BytesIO
import numpy as np
import base64
import re

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)
# Load the model
model = load_model("data/keras_model.h5", compile=False)
# Load the labels
class_names = open("data/labels.txt", "r").readlines()


app = Flask(__name__)


@app.route("/recognize", methods=["POST"])
def recognize():
    req = request.get_json()
    imageBase64 = req['image']
    data = base64.b64decode(imageBase64)
    image = Image.frombytes("RGBA", (1280, 720), data).convert("RGB")
    # image = Image.open(BytesIO(
    #     base64.b64decode(re.sub('^data:image/.+;base64,', '', imageBase64))
    # )).convert("RGB")
    
    # Create the array of the right shape to feed into the keras model
    # The 'length' or number of images you can put into the array is
    # determined by the first position in the shape tuple, in this case 1
    data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
    # resizing the image to be at least 224x224 and then cropping from the center
    size = (224, 224)
    image = ImageOps.fit(image, size, Image.Resampling.LANCZOS)

    # turn the image into a numpy array
    image_array = np.asarray(image)

    # Normalize the image
    normalized_image_array = (image_array.astype(np.float32) / 127.5) - 1

    # Load the image into the array
    data[0] = normalized_image_array

    # Predicts the model
    prediction = model.predict(data)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]


    # Print prediction and confidence score
    print("Class:", class_name[2:], end="")
    print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")

    return {"label": class_name.strip().split()[-1], "confidence": round(float(confidence_score), 3)}


if __name__ == '__main__':
    app.run(debug=False, host='0.0.0.0', port=5005)
