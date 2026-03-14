import torch

print(torch.cuda.is_available())
print(torch.__version__)

# unfused_model = YOLOE("yoloe-v8l.yaml")
# unfused_model.load("pretrain/yoloe-v8l-seg.pt")
# unfused_model.eval()
# unfused_model.cuda()

# with open('./tools/ram_tag_list.txt', 'r') as f:
#     names = [x.strip() for x in f.readlines()]
# vocab = unfused_model.get_vocab(names)
from ultralytics import YOLOE
from PIL import Image

img_path   = "/home/gwq/Pictures/壁纸/rbQB21.jpg"
model_path = "./pretrain/yoloe-11s-seg-pf.pt"

model = YOLOE(model_path).cuda()
# model.set_classes(["Hollow Red Rectangular Frame"], 
#                   model.get_text_pe(["Hollow Red Rectangular Frame"]))
model.eval()

# 使用opencv打开图像，并等比例缩放，长边640
import cv2
import numpy as np

image = cv2.imread(img_path)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
h, w, _ = image.shape
if h > w:
    image = cv2.resize(image, (640, int(640 * h / w)))
else:
    image = cv2.resize(image, (int(640 * w / h), 640))

results = model.predict(image, save=True, verbose=True)

print(results[0].masks.cpu().data.numpy().size)

# import time
# for i in range(100):
#     t = time.time()
#     model.predict(image, save=False)
#     print("predict time spent: ", (time.time() - t) * 1e3, " ms\n")
