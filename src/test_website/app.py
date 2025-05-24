from flask import Flask, render_template, request, redirect, session, Response, url_for
from flask_sqlalchemy import SQLAlchemy
from flask_socketio import SocketIO, emit
from werkzeug.security import check_password_hash
import base64
import cv2
import numpy as np
from datetime import datetime
from flask import flash

app = Flask(__name__)
app.secret_key = 'secret_key_for_session'  # 세션을 위한 시크릿 키 설정

# DB 연결 설정 (SQLite 사용)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:////home/seokhwan/test_website/test.db'
db = SQLAlchemy(app)

# SocketIO 초기화 (다른 컴퓨터와 통신 시 CORS 허용 필수)
socketio = SocketIO(app, cors_allowed_origins="*")

# 사용자 테이블 모델 정의
class User(db.Model):
    __tablename__ = 'users'
    id = db.Column(db.String, primary_key=True)
    password = db.Column(db.String, nullable=False)

# 택배 로그 테이블 모델 정의
###최종 데모 전에는 로그 테이블 삭제하고 다시 할 필요 잇음~
class DeliveryLog(db.Model):
    __tablename__ = 'delivery_logs'
    robot_name = db.Column(db.String, nullable=False)              # 로봇 이름
    parcel_id = db.Column(db.String, primary_key=True)             # 📦 택배 ID (이제 기본키)
    room_number = db.Column(db.String, nullable=False)             # 목표 호실
    received_time = db.Column(db.String, nullable=False)           # 받은 시간
    delivered_time = db.Column(db.String, nullable=False)          # 배달 완료 시간

# 루트 페이지 → 로그인 페이지로 리다이렉트
@app.route('/')
def index():
    return redirect('/login')

# 로그인 처리
@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        uid = request.form['username']
        pw = request.form['password']

        user = User.query.filter_by(id=uid).first()
        if user and user.password == pw:
            session['user'] = uid
            return redirect('/dashboard')
        else:
            flash("❌ 로그인 실패: 아이디 또는 비밀번호가 잘못되었습니다.", "error")
            return redirect('/login')
    return render_template('login.html')

# 대시보드 페이지 (로그인된 경우에만 접근 가능)
@app.route('/dashboard')
def dashboard():
    if 'user' in session:
        return render_template('dashboard.html', username=session['user'])
    return redirect('/login')

# 로그 페이지 (DB에 저장된 배송 로그 조회)
@app.route('/logs')
def logs():
    logs = DeliveryLog.query.order_by(DeliveryLog.received_time.desc()).all()
    return render_template('logs.html', logs=logs)

# 로그아웃 처리
@app.route('/logout')
def logout():
    session.pop('user', None)
    return redirect('/login')

# 카메라 1 영상 수신 및 브로드캐스트
@socketio.on('image_robot8')
def handle_image_cam1(data):
    emit('image_broadcast_cam1', data, broadcast=True)

# 카메라 2 영상 수신 및 브로드캐스트
@socketio.on('image_robot9')
def handle_image_cam2(data):
    emit('image_broadcast_cam2', data, broadcast=True)

# 택배 로봇 로그 수신 시 DB 저장 및 클라이언트 브로드캐스트
"""goal_manager에서 이런식으로 보내줌
self.sio.emit('robot_delivery_log', {
    'robot_name': 'TB4',
    'parcel_id' : 'A',.
    'room_number': self.qr_id,
    'received_time': '2025-05-20 14:22:00',  # 
    'delivered_time': '-' 
})
"""
@socketio.on('robot_delivery_log')
def handle_robot_data(info):
    # DB 저장
    print('📦 받은 데이터:', info)  # 여기에 parcel_id 나오는지 확인
    log = DeliveryLog(
        robot_name=info['robot_name'],
        parcel_id=info['parcel_id'],  
        room_number=info['room_number'],
        received_time=info['received_time'],
        delivered_time=info['delivered_time']
    )
    db.session.add(log)
    db.session.commit()

    # 클라이언트에 실시간 브로드캐스트
    emit('delivery_log_update', info, broadcast=True)

# 서버 실행
if __name__ == '__main__':
    with app.app_context():
        db.create_all()  # 테이블이 없으면 생성
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
