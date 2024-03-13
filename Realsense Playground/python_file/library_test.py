import numpy as np
import matplotlib.pyplot as plt
import cv2

# marker class to package all markers into an object
class marker:

    # Initilzating the main object for marker classes
    # Handle all the infomation coming from 
    def __init__(self , id , corner , distance = None , mask = None):
        
        # initilzating id and marker corners for this 
        self.id = id
        self.corner = corner
        self.distance = distance
        self.mask = mask

    
    # Adding distance variable 
    def add_distance(self, distance):

        # append distance variable
        self.distance = distance

    # Adding mask object
    def add_mask(self, mask):

        # append mask object
        self.mask = mask

    # return distance object
    def get_distance(self):

        # return the object
        return self.distance
    
    # return mask object
    def get_mask(self):

        # return the mask
        return self.mask

    


