#!/usr/bin/env python
import numpy as np
import pickle
import rospy
import time

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
       'biscuits',
       'soap2',
       'soap',
       'book',
       'glue',
       'sticky_notes',
       'snacks',
       'eraser']

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []
    j = 0
    t0 = time.time()
    for model_name in models:
        spawn_model(model_name)
        j = j+1
        tj = time.time()
        print('Capturing Features for model ',j,' of ',len(models),': ',model_name)
        nspawn = 50        
        for i in range(nspawn):
            print('{} of {}').format(i+1, nspawn)
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected on try count',try_count)
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, nbins=64, bins_range = (0,1024), using_hsv=True)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals,nbins=64, bins_range = (0,1024))
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

        delete_model()
        print(time.time()-tj)

    print('Total Elapsed time: ',time.time()-t0)
    pickle.dump(labeled_features, open('training_set_pr2.sav', 'wb'))

