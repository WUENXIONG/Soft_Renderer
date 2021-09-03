#include <vector>
#include <limits>
#include <iostream>
#include <algorithm>
#include "./tool/tgaimage.h"
#include "./tool/model.h"
#include "./tool/geometry.h"
#include "./shader/shader.h"
#include "./widget/win32.h"
#include"./Camera.h"





//���Ǵ��ڵĿ��
const int WINDOW_WIDTH  = 1600;
const int WINDOW_HEIGHT = 1200;

Vec3f     lg(0,0,1);

//����������������lookat�����
Vec3f       eye(0.6, 0.7, 1.75);
Vec3f    center(0,0,0);
Vec3f        up(0,1,0);


int main() {

    float* zbuffer = (float*)malloc(sizeof(float) * WINDOW_WIDTH * WINDOW_HEIGHT);
    unsigned char* framebuffer = (unsigned char*)malloc(sizeof(unsigned char) * WINDOW_WIDTH * WINDOW_HEIGHT * 4);
    for (int i = WINDOW_WIDTH * WINDOW_HEIGHT; i--; zbuffer[i] = (std::numeric_limits<float>::max)());
    memset(framebuffer, 0, sizeof(unsigned char) * WINDOW_WIDTH * WINDOW_HEIGHT * 4);
    window_init(WINDOW_WIDTH, WINDOW_HEIGHT, "Renderer");


    Camera camera(eye,center);

    //�任����
    //lookat(camera.eye,camera.origin, up);
    viewport(WINDOW_WIDTH/8, WINDOW_HEIGHT/8, WINDOW_WIDTH*3/4, WINDOW_HEIGHT*3/4);
    projection(60, (float)(WINDOW_WIDTH)/WINDOW_HEIGHT, -0.1, -10000);
    
    //�任��Ĺ��գ����ﵥλ����Ϊ�˺��������ɫ����
    lg = proj<3>((Projection*ModelView*embed<4>(lg, 0.f))).normalize();
    
    int n = 7;//OBJģ�͸�����Ȼ���ģ��·����ӵ�����
    Model* model[]{
     new Model("D:/VisualStudio_WorkSpace/SRender-master/obj/yayi/yayiarmour1.obj"),
     new Model("D:/VisualStudio_WorkSpace/SRender-master/obj/yayi/yayiarmour2.obj"),
     new Model("D:/VisualStudio_WorkSpace/SRender-master/obj/yayi/yayibody.obj"),
     new Model("D:/VisualStudio_WorkSpace/SRender-master/obj/yayi/yayidecoration.obj"),
     new Model("D:/VisualStudio_WorkSpace/SRender-master/obj/yayi/yayiface.obj"),
     new Model("D:/VisualStudio_WorkSpace/SRender-master/obj/yayi/yayihair.obj"),
     new Model("D:/VisualStudio_WorkSpace/SRender-master/obj/yayi/yayisword.obj")
    };
    
    PhongShader s ;

    while(!window->is_close) { 
        //��ʼ������
        for (int i = WINDOW_WIDTH * WINDOW_HEIGHT; i--; zbuffer[i] = (std::numeric_limits<float>::max)());
        memset(framebuffer, 0, sizeof(unsigned char) * WINDOW_WIDTH * WINDOW_HEIGHT * 4);

        handle(camera);
        eye = camera.eye;
        lookat(camera.eye, camera.origin, up);
        
        for(int k = 0;k<n;k++)
            for (int i = 0; i < model[k]->nfaces(); i++) {
                for (int j = 0; j < 3; j++) 
                    clip_prepared(i, j, model[k]);//����������ʼ�ȴ��ü�������������ꡢ�ü����ꡢ����������������
                    int num = clipping();//�ü��õ��µĶ���Ͷ����Ӧ���������ꡢ�ü����ꡢ����������������
                    
                    //�ѵõ��Ķ�������������������β��ͽ�ȥ��դ��
                    for (int i = 0; i < num - 2; i++) {
                        for (int j = 0; j < 3; j++)
                            s.vertex(i + j,j);

                        triangle(s.varying_tri, s, framebuffer, zbuffer, model[k]);//��դ��
                    }
                    //����任
                    //����Ѳü���ĵ��ͽ�ȥ��դ���������ĵ���ܲ�����������Ҫ����Ū��forѭ�����°���Щ�����������
                
                
            }
            
        window->mouse_info.wheel_delta = 0;
        window->mouse_info.orbit_delta = vec2(0, 0);
        window->mouse_info.fv_delta = vec2(0, 0);
        window_draw(framebuffer);
        msg_dispatch();
    }
    
    for (int i = 0; i < n; i++) delete model[i];
    free(zbuffer);
    free(framebuffer);
    window_destroy();
    return 0;
}

