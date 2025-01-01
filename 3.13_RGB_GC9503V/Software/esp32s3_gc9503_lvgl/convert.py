from PIL import Image
import os

def resize_images(target_width=376, target_height=960):
    # 获取当前文件夹路径
    current_dir = os.getcwd()

    # 新建保存 resized 图片的文件夹
    output_dir = os.path.join(current_dir, "resized_images")
    os.makedirs(output_dir, exist_ok=True)  # 如果文件夹不存在则创建

    # 遍历当前文件夹中的所有文件
    for filename in os.listdir(current_dir):
        # 检查文件是否为 JPG 图片
        if filename.lower().endswith('.jpg'):
            try:
                # 打开图片并打印原始尺寸
                img = Image.open(filename)
                print(f"Processing '{filename}' - Original size: {img.size}")

                # 使用 LANCZOS 高质量缩放算法
                resized_img = img.resize((target_width, target_height), Image.Resampling.LANCZOS)

                # 保存到新文件夹，保持文件名不变
                output_path = os.path.join(output_dir, filename)
                resized_img.save(output_path, quality=95)
                print(f"'{filename}' resized and saved to '{output_path}'")

            except Exception as e:
                print(f"Error processing '{filename}': {e}")

if __name__ == "__main__":
    resize_images()
