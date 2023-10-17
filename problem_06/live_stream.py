import cv2
import time
import fer

# Create a VideoCapture object to capture video from the webcam
cap = cv2.VideoCapture(0)

# Set the frame width and height to 640x480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Create a VideoWriter object to save the video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

# Start an infinite loop to continuously capture frames from the webcam
start_time = time.time()
while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    # Write the frame to the VideoWriter object
    out.write(frame)

    # Wait for 5 seconds to capture 5 seconds of video
    if time.time() - start_time >= 5:
        break

# Stop writing to the VideoWriter object
out.release()

# Use the fer library to process the saved video and detect emotions in faces
detector = fer.FER()
video = cv2.VideoCapture('output.avi')

# create a dictionary to store the emotion scores
emotions = {'angry': [], 'disgust': [], 'fear': [], 'happy': [], 'sad': [], 'surprise': [], 'neutral': []}

while True:
    ret, frame = video.read()
    if not ret:
        break
    result = detector.detect_emotions(frame)

    # Display the processed video with emotion labels
    for face in result:
        (x, y, w, h) = face['box']
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        for emotion, score in face['emotions'].items():
            text = f"{emotion}: {score:.2f}"
            print(text)
            emotions[emotion].append(score) # store the emotion score in the dictionary
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# create graph of emotions over time
import matplotlib.pyplot as plt

# plot the emotion scores over time
for emotion, scores in emotions.items():
    plt.plot(scores, label=emotion)

plt.legend()
plt.xlabel('Time (frames)')
plt.ylabel('Emotion score')
plt.show()

# Release the video capture and destroy all windows
video.release()
cv2.destroyAllWindows()


# Release the video capture and destroy all windows
video.release()
cv2.destroyAllWindows()
