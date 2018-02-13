import numpy as np
from sklearn.cluster import KMeans
import os
import math

def L2_Normalize(data):
    sum = np.sum(data ** 2,1)
    sqrt = np.sqrt(sum)

    for i in range(data.shape[0]):
        data[i] = data[i] / sqrt[i]

    return data

def CalcNN(centers, desps, k):
    # centers: K D
    # desps: N D

    nn = np.empty((desps.shape[0], centers.shape[0]))

    for i in range(desps.shape[0]):
        desp_tile = np.tile(desps[i], (centers.shape[0],1))
        nn[i] = np.sum((desp_tile - centers) ** 2, axis=1)

    nn = np.sort(nn, axis=-1)
    return nn[:,:2]

def CalcKClusters(conv5_4_Desps, fileCenter):
    estimator = KMeans(n_clusters=64)
    estimator.fit(conv5_4_Desps)
    centers = estimator.cluster_centers_
    np.save(fileCenter, centers)
    print("Cluster centers saved:", fileCenter, centers.shape)
    return centers

def SaveInitValues(working_path_tag, weight, bias, centers):
    prefix = working_path_tag + "_vlad_"
    fname_weight = prefix + "weight.npy"
    fname_bias = prefix + "bias.npy"
    fname_center = prefix + "center.npy"

    np.save(fname_weight, weight)
    np.save(fname_bias, bias)
    np.save(fname_center, centers)

def PrepareInitValues(working_path_tag):
    fileFeature = working_path_tag + "_ft_conv5_4.npy"
    fileCCenter = working_path_tag + "_ccenter.npy"

    if os.path.isfile(fileFeature):
        conv5_4_Desps = np.load(fileFeature)
        conv5_4_Desps = conv5_4_Desps.reshape((-1, conv5_4_Desps.shape[-1]))
        print("Load deep feature from cache:", fileFeature, conv5_4_Desps.shape)

    else:
        print("file doesn't exist:", fileFeature)
        return

    if os.path.isfile(fileCCenter):

        centers = np.load(fileCCenter)
        print("Load centers from file:", fileCCenter, centers.shape)
    else:
        print("No cluster center file, calculating:")
        centers = CalcKClusters(conv5_4_Desps, fileCCenter)
    
    # normalize centers
    # centers = L2_Normalize(centers)

    nn = CalcNN(centers, conv5_4_Desps, 2)
    mean = np.mean(nn,axis=0)
    print(mean)

    alpha = -math.log(0.01)/(mean[1] / mean[0])
    print("alpha is:", alpha)

    weight = centers * 2 * alpha
    bias = -alpha * np.sum(centers ** 2, 1)

    SaveInitValues(working_path_tag, weight, bias, centers)
    return