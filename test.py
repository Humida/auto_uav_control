import cv2
import numpy as np

def process_frame(frame):
    # Chuyển frame sang ảnh xám
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Thực hiện xử lý ảnh tại đây sử dụng CUDA
    
    return gray

def main():
    # Khởi tạo webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Không thể mở webcam")
        return

    # Thiết lập CUDA
    cv2.cuda.setDevice(0)

    while True:
        # Đọc frame từ webcam
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc frame")
            break

        # Tạo GpuMat từ frame
        frame_gpu = cv2.cuda_GpuMat()
        frame_gpu.upload(frame)

        # Thực hiện xử lý frame trên GPU
        gray_gpu = cv2.cuda.cvtColor(frame_gpu, cv2.COLOR_BGR2GRAY)

        # Tải kết quả về CPU
        gray = gray_gpu.download()

        # Hiển thị frame xử lý
        cv2.imshow('Processed Frame', gray)

        # Thoát nếu nhấn phím 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Giải phóng tài nguyên
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()