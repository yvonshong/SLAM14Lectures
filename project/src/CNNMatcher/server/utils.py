import skimage
import skimage.io
import skimage.transform
import numpy as np
import os

class SingleTrainData(object):
    def __init__(self, idx, pos, neg):
        self.idx = idx
        self.pos = pos
        self.neg = neg
    def Print(self):
        print("idx", self.idx)
        print("Pos:")
        for p in self.pos:
            print(p)
        print("Neg:")
        for n in self.neg:
            print(n)

def GetImgNames(datapath, filename):
    print("Load image filenames from: ", filename,"...")
    print("Image data folder: ", datapath, "...")

    file = open(filename, mode = 'r')
    lines = file.readlines()
    imgNameTrain = []
    for line in lines:
        s = line.split()
        if(len(s) > 0):
            imgNameTrain.append(datapath + s[0])
    return imgNameTrain

def GetPosNegIdx(filename):
    print("Load idx data from:", filename)
    file = open(filename)
    lines = file.readlines()
    trainEntries = []
    for i in range(len(lines)):
        s = lines[i].split()
        pos = []
        neg = []
        if(len(s) > 0):
            pos.append(int(s[0]))
            pos.append(int(s[1]))
            pos.append(int(s[2]))
            pos.append(int(s[3]))

            neg.append(int(s[4]))
            neg.append(int(s[5]))
            neg.append(int(s[6]))
            neg.append(int(s[7]))

            data = SingleTrainData(i, pos, neg)
            trainEntries.append(data)
    return trainEntries

# returns image of shape [224, 224, 3]
# [height, width, depth]
def loadImg(path):
    # load image
    img = skimage.io.imread(path)
    img = img / 255.0
    assert (0 <= img).all() and (img <= 1.0).all()
    # print "Original Image Shape: ", img.shape
    # we crop image from center
    short_edge = min(img.shape[:2])
    yy = int((img.shape[0] - short_edge) / 2)
    xx = int((img.shape[1] - short_edge) / 2)
    crop_img = img[yy: yy + short_edge, xx: xx + short_edge]
    # resize to 224, 224
    resized_img = skimage.transform.resize(crop_img, (224, 224))
    return resized_img

def LoadInitVal(working_path_tag):
    prefix = working_path_tag + "_vlad_"
    fname_weight = prefix + "weight.npy"
    fname_bias = prefix + "bias.npy"
    fname_center = prefix + "center.npy"

    print("Load init values from file:")
    print(fname_weight)
    print(fname_bias)
    print(fname_center)

    vlad_w = np.load(fname_weight)
    vlad_b = np.load(fname_bias)
    vlad_c = np.load(fname_center)

    return vlad_w, vlad_b, vlad_c

def LoadImgsCached(working_path_tag, imgNames):

    fnameImg = working_path_tag + "_imgs.npy"
    # load imgs
    allImgs = []
    if os.path.isfile(fnameImg):
        allImgs = np.load(fnameImg)
        print("Load imgs from cache: ", fnameImg, allImgs.shape)
    else:
        print("Load imgs from raw files")
        for i in range(len(imgNames)):
            print("Load:", i+1, " / ", len(imgNames))
            allImgs.append(loadImg(imgNames[i]))
        allImgs = np.array(allImgs)
        print(allImgs.shape)
        np.save(fnameImg, allImgs)

    return allImgs