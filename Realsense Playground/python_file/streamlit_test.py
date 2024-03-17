import streamlit as st
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

#st.title("live stream test")

img_ph = st.empty()

stop_button = st.button("stop")

while cap.isOpened() and not stop_button:
    
    # 
    ret , frame = cap.read()

    if not ret:
        st.write("video capture is ended")

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    img_ph.image(frame , channels = "RGB")

    if cv2.waitKey(1) & 0XFF == ord('q') or stop_button:
        break

cap.release()
cv2.destroyAllWindows()