#include"./Camera.h"
#include"./widget/win32.h"

Camera::Camera(Vec3f e, Vec3f org) {
	eye = e;
	origin = org;

}
Camera::~Camera() {}

void update(Camera& camera) {
	
	//我们认为模型处于相机和相机坐标系所在参考坐标系原点，相机坐标系原点和相机位置的坐标都是相对于参考坐标系的
	
	//相机位置与相机坐标系原点的方向
	Vec3f distance = camera.eye - camera.origin;
	
	//鼠标按下左键时左右移动的距离，用来控制相机左右旋转
	float lr = window->mouse_info.orbit_delta[0];
	//鼠标按下左键时上下移动的距离，用来控制相机上下旋转
	float ud = window->mouse_info.orbit_delta[1];
	//鼠标滚轮移动的距离，用来控制相机与相机坐标系的距离
	float zoom = -window->mouse_info.wheel_delta;

	float dis = distance.norm();//当前相机与相机坐标系的距离
	float yaw = atan2(distance[0], distance[2]);//当前方向在x-z平面(模型的)的投影与z轴夹角
	float pitch = acos(distance[1] / dis);//当前方向与x-z(模型的)平面的夹角

	//将鼠标移动的数据更新到dis、yaw、pitch上
	dis += 0.2 * zoom; 
	yaw += 0.01 * lr;
	pitch += 0.01 * ud;

	//防止在上下旋转时，模型倒置
	if (pitch > PI) pitch = PI - 0.0001;
	if (pitch < 0) pitch = 0.0001;

	//更新相机位置
	camera.eye[0] = camera.origin[0] + dis * sin(yaw) * sin(pitch);
	camera.eye[1] = camera.origin[1] + dis * cos(pitch);
	camera.eye[2] = camera.origin[2] + dis * sin(pitch) * cos(yaw);

}

void handle(Camera& camera) {
	//键盘输入，用来上下左右平移相机
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

	//鼠标输入
	if (window->buttons[0]) {
		vec2 pos = get_mouse_pos();
		window->mouse_info.orbit_delta = window->mouse_info.orbit_pos - pos;
		window->mouse_info.orbit_pos = pos;
	}

	//更新
	update(camera);
}