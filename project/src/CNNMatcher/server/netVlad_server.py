from concurrent import futures
import time
import skimage
import skimage.io
import skimage.transform
import tensorflow as tf
import numpy as np
import vggVlad
import utils
import featureCluster
import cv2
import grpc
import netVlad_pb2
import netVlad_pb2_grpc
import getopt, sys

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

class NetConnect(netVlad_pb2_grpc.NetConnectServicer):
    def img2vec(self, request, context):
        imgNames = []
        allImgs = []
        img_decode = []
        npimg = np.fromstring(request.img, dtype=np.uint8)
        img_decode = cv2.imdecode(npimg, 1)
        allImgs.append(img_decode)
        allImgs = np.array(allImgs)

        for i in range(allImgs.shape[0]):
            k = sess.run(net.reduceVec, feed_dict={input: [allImgs[i]]})
            vec = k[0]
        flag = "success"
        return netVlad_pb2.NetResponse(flag=flag, vec=vec)


def serve(argv):
    try:  
        opts,args = getopt.getopt(argv,'hi:',['help','input.npy='])
        for opt,arg in opts:  
            if opt in ("-h", "--help"):  
                print("input like $ python netVlad_server.py /path/to/vgg19.npy")
                sys.exit(1) 
            elif opt in ("-i", "--input"):  
                print("for test option") 
                file_vgg_npy = arg
    except getopt.GetoptError:  
         print("parameter error!")
         print("input like $ python netVlad_server.py /path/to/vgg19.npy")
    # Init net structure
    global graphdefult
    graphdefult = tf.Graph()
    with graphdefult.as_default():
        global input
        input = tf.placeholder(tf.float32, [1, 224, 224, 3])
        global net
        net = vggVlad.vggVlad(file_vgg_npy)  #use GPU
        net.build(input)  #use GPU
        # Set GPU options
        global gpu_options
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.75)

    global sess
    with tf.Session(
            graph=graphdefult,
            config=tf.ConfigProto(gpu_options=gpu_options)) as sess:
        sess.run(tf.global_variables_initializer())
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        netVlad_pb2_grpc.add_NetConnectServicer_to_server(NetConnect(), server)
        server.add_insecure_port('[::]:50051')
        server.start()
        try:
            while True:
                time.sleep(_ONE_DAY_IN_SECONDS)
        except KeyboardInterrupt:
            server.stop(0)

if __name__ == '__main__':
	serve(sys.argv[1:])
