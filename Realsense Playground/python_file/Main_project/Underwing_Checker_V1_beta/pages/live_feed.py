import streamlit as st

#st.title("Place holder V1")

def show(frame):

    st.title("Place holder V1")

    frame_placeholder = st.empty
    
    if frame:
        frame_placeholder.image(frame , Channels = "BGR")