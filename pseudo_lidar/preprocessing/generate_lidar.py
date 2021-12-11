import argparse
import os

import numpy as np
from numpy.core.fromnumeric import shape
from numpy.lib.function_base import disp
import scipy.misc as ssc

import kitti_util
import cv2

def project_disp_to_points(calib, disp, max_high):
    disp[disp < 0] = 0
    baseline = 0.54
    mask = disp > 0
    depth = calib.f_u * baseline / (disp + 1. - mask)
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows))
    points = np.stack([c, r, depth])
    points = points.reshape((3, -1))
    points = points.T
    points = points[mask.reshape(-1)]
    cloud = calib.project_image_to_velo(points)
    return cloud
    valid = (cloud[:, 0] >= 0) & (cloud[:, 2] < max_high)
    return cloud[valid]

def project_depth_to_points(calib, depth, max_high):
    if len(depth.shape) == 4:
        depth = depth.squeeze(0)
        depth = depth.squeeze(0)
        
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows))
    points = np.stack([c, r, depth])
    points = points.reshape((3, -1))
    points = points.T
    cloud = calib.project_image_to_velo(points)
    return cloud
    valid = (cloud[:, 0] >= 0) & (cloud[:, 2] < max_high)
    return cloud[valid]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate Libar')
    parser.add_argument('--calib_path', type=str,
                        default='~/Kitti/object/training/calib')
    parser.add_argument('--input_path', type=str,
                        default='~/Kitti/object/training/predicted_disparity')
    parser.add_argument('--output_path', type=str,
                        default='~/Kitti/object/training/predicted_velodyne')
    parser.add_argument('--image_raw_path', type=str,
                        default='~/Kitti/object/training/predicted_disparity')                        
    parser.add_argument('--max_high', type=int, default=1)
    parser.add_argument('--is_depth', action='store_true')

    args = parser.parse_args()

    assert os.path.isdir(args.input_path)
    assert os.path.isdir(args.calib_path)

    if not os.path.isdir(args.output_path):
        os.makedirs(args.output_path)

    disps = [x for x in os.listdir(args.input_path) if x[-3:] == 'png' or x[-3:] == 'npy']
    disps = sorted(disps)

    calib_file = '{}/{}.txt'.format(args.calib_path, 'calib')

    for fn in disps:
        predix = fn[:-4]
        
        calib = kitti_util.Calibration(calib_file)
        # disp_map = ssc.imread(args.disparity_dir + '/' + fn) / 256.
        if fn[-3:] == 'png':
            disp_map = cv2.imread(args.input_path + '/' + fn)
            disp_map = disp_map[:,:,0]
        elif fn[-3:] == 'npy':
            disp_map = np.load(args.input_path + '/' + fn)
        else:
            assert False
        
        width, height = disp_map.shape[3], disp_map.shape[2]
        image_raw_fn = args.image_raw_path+predix+'.png'
        image_raw = cv2.imread(image_raw_fn)

        if image_raw.shape[1] != width or image_raw.shape[0] == height:
        
            image_raw = cv2.resize(image_raw, (width,height), interpolation=cv2.INTER_LANCZOS4)
            # image_raw = image_raw.reshape(image_raw.shape[1], -1, 3)
        image_raw = image_raw.reshape(-1,3)
        
        if not args.is_depth:
            disp_map = (disp_map * 256).astype(np.uint16)/256.
            lidar = project_disp_to_points(calib, disp_map, args.max_high)
        else:
            disp_map = (disp_map).astype(np.float32)/256.
            lidar = project_depth_to_points(calib, disp_map, args.max_high)

        # pad 1 in the indensity dimension
        lidar = np.concatenate([lidar, np.ones((lidar.shape[0], 3))], 1)
        lidar[:,3]= image_raw[:,0]
        lidar[:,4]= image_raw[:,1]
        lidar[:,5]= image_raw[:,2]
        
        lidar = lidar.astype(np.float32)
        lidar.tofile('{}/{}.bin'.format(args.output_path, predix))
        print('Finish Depth {}'.format(predix))
