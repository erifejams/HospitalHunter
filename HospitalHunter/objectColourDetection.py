#####Using this to check the assciated colour of the objects in the scene
###and for testing purposes to see if it works for a normal image
import cv2
import pandas as pd

##aim
##robot keeps searching until it finds the colour
##when robot finds the colour check what object it is
##say find the bed etc


##change to the path of the image where the data from the camera will be stored
img_path = r'/home/robotics23/dev_ws/src/HospitalHunter/data/dog.jpg'
img = cv2.imread(img_path)

# declaring global variables (are used later on)
clicked = False
r = g = b = x_pos = y_pos = 0

# Reading csv file with pandas and giving names to each column
index = ["color", "color_name", "hex", "R", "G", "B"]
csv = pd.read_csv('./data/colors.csv', names=index, header=None)

#print the dataset
#print(csv)


# function to calculate minimum distance from all colors and get the most matching color
def get_color_name(R, G, B):
    minimum = 10000
    for i in range(len(csv)):
        d = abs(R - int(csv.loc[i, "R"])) + abs(G - int(csv.loc[i, "G"])) + abs(B - int(csv.loc[i, "B"]))
        if d <= minimum:
            minimum = d
            cname = csv.loc[i, "color_name"]
    return cname


# # function to get x,y coordinates of mouse double click
# def draw_function(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDBLCLK:
#         global b, g, r, x_pos, y_pos, clicked
#         clicked = True
#         x_pos = x
#         y_pos = y
#         b, g, r = img[y, x]
#         b = int(b)
#         g = int(g)
#         r = int(r)


# cv2.namedWindow('image')
# cv2.setMouseCallback('image', draw_function)

# while True:

#     cv2.imshow("image", img)
#     if clicked:

#         # cv2.rectangle(image, start point, endpoint, color, thickness)-1 fills entire rectangle
#         cv2.rectangle(img, (20, 20), (750, 60), (b, g, r), -1)

#         # Creating text string to display( Color name and RGB values )
#         text = get_color_name(r, g, b) + ' R=' + str(r) + ' G=' + str(g) + ' B=' + str(b)

#         # cv2.putText(img,text,start,font(0-7),fontScale,color,thickness,lineType )
#         cv2.putText(img, text, (50, 50), 2, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

#         # For very light colours we will display text in black colour
#         if r + g + b >= 600:
#             cv2.putText(img, text, (50, 50), 2, 0.8, (0, 0, 0), 2, cv2.LINE_AA)

#         clicked = False

#     # Break the loop when user hits 'esc' key
#     if cv2.waitKey(20) & 0xFF == 27:
#         break

# cv2.destroyAllWindows()
##checking that the function get colour works
result = get_color_name(60, 60, 60)
print(f'this colour is {result}')

# colourData = [['Palatinate Purple', 'Purple Circle Shape', '85, 39, 100'],
#                ['Stormcloud','Hospital Bed', '86, 100, 98'],
#                ['Caput Mortuum', 'Orange Circle Shape', '85, 39, 0'],
#                ['Spring Bud', 'Green Circle Shape ', '16, 39, 0'],
#                ['Rosewood', 'Blood Bag Stand', '100, 0, 0'],
#                ['Black Olive', 'Machine Checker', '60, 60, 60'], 
#                ['Gray-Asparagus', 'Surgery Bed', '77, 100, 66'],
#                ['Chocolate (Traditional)', 'Yellow Circle Shape', '85, 71, 0']] ###stores the colour of the the objects in the scene

# ObjectDataset = pd.DataFrame(colourData,columns = ['Colours', 'Object', 'RGB'])
# print(ObjectDataset)