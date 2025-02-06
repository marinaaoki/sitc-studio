#!/usr/bin/env python2
import cv2
import os

kitchen_sensor = 2
person_id = 5

activities = [
    "B1_BED_OUT",
    "B2_BED_OUT",
    "B1_JACKET_ON",
    "B2_JACKET_ON",
    "_FRIDGE_OPEN",
    "_FRIDGE_CLOSE",
    "_PREPARE_MEAL",
    "D1_WATER",
    "D2_WATER",
    "D1_EAT",
    "D2_EAT",
    "L1_SIT_DOWN",
    "L2_SIT_DOWN",
    "L1_STAND_UP",
    "L2_STAND_UP",
    "L1_FALL_DOWN",
    "L2_FALL_DOWN",
    "E1_SHOES_ON",
    "E1_LEAVE_HOUSE",
    "E1_ENTER_HOUSE",
    "E1_SHOES_OFF",
    "B1_JACKET_OFF",
    "B2_JACKET_OFF",
    "B1_BED_IN",
    "B2_BED_IN",
]

for activity in activities:
    if activity.startswith("_"):
        activity = "K" + str(kitchen_sensor) + activity
    print("Activity: " + activity)

    image_dir = '/media/ubi-lab-desktop/disk6s2/sitc_data/Person00' + str(person_id) + '/' + activity + '/depthImages/'

    # display the images with frame id overlaid and wait for user to press a key
    images = os.listdir(image_dir)

    # press arrow keys to flip through images
    i = 0
    while True:
        cv_image = cv2.imread(image_dir + images[i])
        # show frame id in top left corner
        cv2.putText(cv_image, images[i], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Depth Image", cv_image)
        key = cv2.waitKey(0)

        # if key is 'b', then append a line to the file with text 'begin': frame id
        if key == 98:
            # delete all images before this one
            for j in range(i):
                os.remove(image_dir + images[j])
            print("Begin: " + images[i])
            i += 1
        # if key is 'e', then append a line to the file with text 'end': frame id
        elif key == 101:
            # delete all images after this one and break
            for j in range(i+1, len(images)):
                os.remove(image_dir + images[j])
            print("End: " + images[i])
            break

        # if key is 'q', then quit
        elif key == 113 or key == 3:
            break

        # if key is left arrow, go to previous image
        elif key == 81:
            i = max(i - 1, 0)

        # if key is right arrow, go to next image
        elif key == 83:
            i = min(i + 1, len(images) - 1)
    cv2.destroyAllWindows()


    
