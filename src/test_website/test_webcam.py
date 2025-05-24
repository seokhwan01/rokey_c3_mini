import cv2

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ 웹캠을 열 수 없습니다.")
        return

    print("✅ 웹캠 연결됨. ESC를 누르면 종료합니다.")
    cv2.namedWindow("📷 Webcam View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("📷 Webcam View", 640, 480)
    cv2.moveWindow("📷 Webcam View", 100, 100)

    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            print("⚠️ 프레임을 읽지 못했습니다.")
            continue  # 또는 break로 종료

        print("✅ frame size:", frame.shape)
        cv2.imshow("📷 Webcam View", frame)

        if cv2.waitKey(1) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
