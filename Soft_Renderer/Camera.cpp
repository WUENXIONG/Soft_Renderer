#include"./Camera.h"
#include"./widget/win32.h"

Camera::Camera(Vec3f e, Vec3f org) {
	eye = e;
	origin = org;

}
Camera::~Camera() {}

void update(Camera& camera) {
	
	//������Ϊģ�ʹ���������������ϵ���ڲο�����ϵԭ�㣬�������ϵԭ������λ�õ����궼������ڲο�����ϵ��
	
	//���λ�����������ϵԭ��ķ���
	Vec3f distance = camera.eye - camera.origin;
	
	//��갴�����ʱ�����ƶ��ľ��룬�����������������ת
	float lr = window->mouse_info.orbit_delta[0];
	//��갴�����ʱ�����ƶ��ľ��룬�����������������ת
	float ud = window->mouse_info.orbit_delta[1];
	//�������ƶ��ľ��룬��������������������ϵ�ľ���
	float zoom = -window->mouse_info.wheel_delta;

	float dis = distance.norm();//��ǰ������������ϵ�ľ���
	float yaw = atan2(distance[0], distance[2]);//��ǰ������x-zƽ��(ģ�͵�)��ͶӰ��z��н�
	float pitch = acos(distance[1] / dis);//��ǰ������x-z(ģ�͵�)ƽ��ļн�

	//������ƶ������ݸ��µ�dis��yaw��pitch��
	dis += 0.2 * zoom; 
	yaw += 0.01 * lr;
	pitch += 0.01 * ud;

	//��ֹ��������תʱ��ģ�͵���
	if (pitch > PI) pitch = PI - 0.0001;
	if (pitch < 0) pitch = 0.0001;

	//�������λ��
	camera.eye[0] = camera.origin[0] + dis * sin(yaw) * sin(pitch);
	camera.eye[1] = camera.origin[1] + dis * cos(pitch);
	camera.eye[2] = camera.origin[2] + dis * sin(pitch) * cos(yaw);

}

void handle(Camera& camera) {
	//�������룬������������ƽ�����
	camera.c_z = (camera.eye - camera.origin).normalize();
	camera.c_x = cross(up, camera.c_z).normalize();
	camera.c_y = cross(camera.c_z, camera.c_x).normalize();

	if (window->keys['W']){
		camera.eye = camera.eye + 0.2f * camera.c_y;//-10.0 / window->width * camera.z*distance;
		camera.origin = camera.origin + 0.2f * camera.c_y;
	}

	if (window->keys['S']){
		camera.eye = camera.eye -0.2f * camera.c_y;
		camera.origin = camera.origin -0.2f * camera.c_y;
	}

	if (window->keys['A']) {
		camera.eye = camera.eye -0.2f * camera.c_x;//-10.0 / window->width * camera.z*distance;
		camera.origin = camera.origin -0.2f * camera.c_x;
	}

	if (window->keys['D']) {
		camera.eye = camera.eye + 0.2f * camera.c_x;//-10.0 / window->width * camera.z*distance;
		camera.origin = camera.origin + 0.2f * camera.c_x;
	}

	if (window->keys[VK_ESCAPE]){
		window->is_close = 1;
	}

	//�������
	if (window->buttons[0]) {
		vec2 pos = get_mouse_pos();
		window->mouse_info.orbit_delta = window->mouse_info.orbit_pos - pos;
		window->mouse_info.orbit_pos = pos;
	}

	//����
	update(camera);
}