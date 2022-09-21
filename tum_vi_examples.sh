#!/bin/bash
pathDatasetTUM_VI='/work/bj_stereo_fisheye_data/bj_stereo_fisheye_3' #Example, it is necesary to change it by the dataset path


#------------------------------------
# Stereo Examples
echo "Launching Room 1 with Stereo sensor"

./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt "$pathDatasetTUM_VI"/bj_stereo_fisheye.ymal "$pathDatasetTUM_VI"/cam0 "$pathDatasetTUM_VI"/cam1 "$pathDatasetTUM_VI"/bj_stereo_fisheye_time3.txt bj_stereo_fisheye_3


# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt "$pathDatasetTUM_VI"/bj_stereo_fisheye.ymal "$pathDatasetTUM_VI"/cam0 "$pathDatasetTUM_VI"/cam1 "$pathDatasetTUM_VI"/bj_stereo_fisheye_data1.txt bj_stereo_fisheye_1


# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/cam0 "$pathDatasetTUM_VI"/cam1 Examples/Stereo/TUM_TimeStamps/dataset-slides1_512.txt dataset-slides1_512_stereo

# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/cam0 "$pathDatasetTUM_VI"/cam1 Examples/Stereo/TUM_TimeStamps/dataset-room1_512.txt dataset-room1_512_stereo

# echo "Launching Room 2 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room2_512.txt dataset-room2_512_stereo

# echo "Launching Room 3 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room3_512.txt dataset-room3_512_stereo

# echo "Launching Room 4 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room4_512.txt dataset-room4_512_stereo

# echo "Launching Room 5 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room5_512.txt dataset-room5_512_stereo

# echo "Launching Room 6 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room6_512.txt dataset-room6_512_stereo
