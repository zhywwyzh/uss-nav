import torch
from PIL import Image
import mobileclip
import time

# 设置CUDA设备
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# 加载模型并移至GPU
model, _, preprocess = mobileclip.create_model_and_transforms('mobileclip_b', pretrained='./mobileclip_blt.pt')
model = model.to(device)
model.eval()  # 设置为评估模式

tokenizer = mobileclip.get_tokenizer('mobileclip_b')

# 准备文本并移至GPU
text = tokenizer(["desk", "coffee table", "display monitor", "screen", "sofa", "laptop"]).to(device)

with torch.no_grad(), torch.cuda.amp.autocast():
    # 预热运行（GPU首次运行可能较慢）
    for _ in range(5):
        _ = model.encode_text(text)
    
    # 正式计时
    total_time = 0
    for i in range(1):
        t1 = time.time()
        
        text_features = model.encode_text(text)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        text_features = text_features.cpu().numpy()
        # 计算余弦相似度
        cosine_sim = 100 * text_features.dot(text_features.T)
        print(f"Cosine similarity: {cosine_sim}")

        elapsed = (time.time() - t1) * 1000  # 毫秒
        total_time += elapsed
        print(f"Text encoding time: {elapsed:.3f}ms")

    print(f"Average time per iteration: {total_time/100:.3f}ms")
    print(f"Text features shape: {text_features.shape}")
    print(f"Device of text_features: {text_features}")  # 验证输出设备