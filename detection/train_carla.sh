#!/usr/bin/env bash

echo "Training for carla"

PIPELINE_CONFIG_PATH=models/faster_rcnn_resnet101_coco_carla.config
MODEL_DIR=out/model_carla

python deps/tensorflow-models/object_detection/train.py \
    --logtostderr \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --train_dir=${MODEL_DIR}
