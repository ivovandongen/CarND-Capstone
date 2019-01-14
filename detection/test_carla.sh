#!/bin/bash

python src/test_model.py --model out/carla_model_export/frozen_inference_graph.pb --labels models/labelmap.pbtxt --image_path test/carla --output_path out/test/carla --classes 14
