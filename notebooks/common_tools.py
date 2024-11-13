import matplotlib.pyplot as plt
import numpy as np
import cv2

def plt_show(image: np.ndarray, figsize = (5,5)) -> None:

    """Display coloured images using pyplot in jupyter notebook

    Args:
        image (np.ndarray): input image to be displayed
        figsize (tuple, optional): size of the images. Defaults to (5,5).
    """
    # TO DO:
    # make it work with black & white images as well

    plt.figure(figsize=figsize)
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.axis("off")
    plt.show()