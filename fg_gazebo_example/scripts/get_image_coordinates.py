import cv2


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Mouse clicked at (x={x}, y={y})")


def show_image_with_mouse_coordinates(image_path):
    # Load the image
    image = cv2.imread(image_path)

    # Create a window and set the callback function for mouse events
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", on_mouse)

    while True:
        # Display the image
        cv2.imshow("Image", image)

        # Exit loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Clean up and close the window
    cv2.destroyAllWindows()


# Example Usage
image_path = "/home/jatinvira/free_ws/src/ros_manipulator/fg_gazebo_example/bag_out/110589000000.png"
show_image_with_mouse_coordinates(image_path)
