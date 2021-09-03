#include <cmath>
#include <limits>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include<vector>
#include"../tool/maths.h"
#include"../tool/model.h"
#include"../tool/geometry.h"
#include "../tool/macro.h"
#include "./shader.h"
//������ϼ�ctrl-m-o�������з��������Կ���9��88�о���ȫ�ֱ����ͺ�����ʵ��
extern Vec3f lg;
Vec3f light;
Vec3f v;
extern Vec3f eye;
extern const int WINDOW_HEIGHT;
extern const int WINDOW_WIDTH;
Matrix ModelView;
Matrix Viewport;
Matrix Projection;
Vec4f clipcor[10];
Vec3f worcor[10];
Vec3f normal[10];
Vec2f uv[10];






 //��׵��������棬����һ����ֹ�����w
typedef enum 
{
    w,
    left,
    right,
    top,
    bottom,
    near,
    far
} frustrum;

void clip_prepared(int iface, int nthvert, Model* model) {
    uv[nthvert] = (model->uv(iface, nthvert));
    clipcor[nthvert] = Projection * ModelView * embed<4>(model->vert(iface, nthvert));
    normal[nthvert] = proj<3>((Projection * ModelView).invert_transpose() * embed<4>(model->normal(iface, nthvert), 0.f));
    worcor[nthvert] = (model->vert(iface, nthvert));
}

bool inside_frustrum(frustrum plane, Vec4f vert) {
    switch(plane) {
    case w:
        return vert[3] <= -EPSILON;
    case left:
        return vert[0] <= -vert[3];
    case right:
        return vert[0] >= vert[3];
    case top:
        return vert[1] >= vert[3];
    case bottom:
        return vert[1] <= -vert[3];
    case near:
        return vert[2] >= vert[3];
    case far:
        return vert[2] <= -vert[3];
    default:
        return false;
   }

}

float get_ratio(Vec4f out, Vec4f in, frustrum plane) {
    switch (plane)
    {
    case w:
        return (out[3] + EPSILON) / out[3] - in[3];
    case left:
        return (out[3] + out[0]) / (out[3] + out[0]) - (in[3] + in[0]);
    case right:
        return (out[3] - out[0]) / (out[3] - out[0]) - (in[3] - in[0]);
    case top:
        return (out[3] - out[1]) / (out[3] - out[1]) - (in[3] - in[1]);
    case bottom:
        return (out[3] + out[1]) / (out[3] + out[1]) - (in[3] + in[1]);
    case near:
        return (out[3] - out[2]) / (out[3] - out[2]) - (in[3] - in[2]);
    case far:
        return (out[3] + out[2]) / (out[3] + out[2]) - (in[3] + in[2]);
    default:
        return 0;
    }
}

int clip_single_plane(frustrum plane, int& vert_nums) {
    int total_vert = 0;

    Vec4f temp_clipcor[10];
    std::copy(clipcor, (clipcor + 10), temp_clipcor);
    Vec3f temp_worcor[10];
    std::copy(worcor, (worcor + 10), temp_worcor);
    Vec3f temp_normal[10];
    std::copy(normal, (normal + 10), temp_normal);
    Vec2f temp_uv[10];
    std::copy(uv, (uv + 10), temp_uv);

    for (int i = 0; i < vert_nums; i++) {
        int curr_index = i;
        int prev_index = (i - 1 + vert_nums) % vert_nums;
        
        Vec4f curr_vert = temp_clipcor[curr_index];
        Vec4f prev_vert = temp_clipcor[prev_index];
        bool curr_inside = inside_frustrum(plane, curr_vert);
        bool prev_inside = inside_frustrum(plane, prev_vert);
        if (curr_inside != prev_inside) {
            float t = get_ratio(prev_vert, curr_vert, plane);
            clipcor[total_vert] = prev_vert + (curr_vert-prev_vert)*t;
            worcor[total_vert] = temp_worcor[prev_index] + (temp_worcor[curr_index] - temp_worcor[prev_index]) * t;
            normal[total_vert] = temp_normal[prev_index] + (temp_normal[curr_index] - temp_normal[prev_index]) * t;
            uv[total_vert] = temp_uv[prev_index] + (temp_uv[curr_index] - temp_uv[prev_index]) * t;
            total_vert++;
        }
        if (curr_inside) {
            clipcor[total_vert] = temp_clipcor[curr_index];
            worcor[total_vert] = temp_worcor[curr_index];
            normal[total_vert] = temp_normal[curr_index];
            uv[total_vert] = temp_uv[curr_index];
            total_vert++;
        }
        return total_vert;
    }
}

int clipping() { 
    int total_vert = 3;
    for(int i=0;i<5;i++)
        clip_single_plane((frustrum)i, total_vert);
    return total_vert;
    
}

void set_color(unsigned char* framebuffer, int x, int y, TGAColor c) {
    int i;
    int index = ((WINDOW_HEIGHT - y - 1) * WINDOW_WIDTH + x) * 4; // the origin for pixel is bottom-left, but the framebuffer index counts from top-left

    for (i = 0; i < 3; i++)
        framebuffer[index + i] = c.bgra[2 - i];
}

void viewport(int x, int y, int w, int h) {
    Viewport = Matrix::identity();
    Viewport[0][3] = x + w / 2.f;
    Viewport[1][3] = y + h / 2.f;
    Viewport[2][3] = 0;
    Viewport[0][0] = w / 2.f;
    Viewport[1][1] = h / 2.f;
    Viewport[2][2] = 0;
}

void projection(float fovy, float aspect, float near, float far) {
    Projection = Matrix::identity();
    fovy = fovy / 180.0 * PI;
    float t = fabs(near) * tan(fovy / 2);
    float r = aspect * t;

    Projection[0][0] = near / r;
    Projection[1][1] = near / t;
    Projection[2][2] = (near + far) / (near - far);
    Projection[2][3] = 2 * near * far / (far - near);
    Projection[3][2] = 1;
    Projection[3][3] = 0;
}

void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye - center).normalize();
    Vec3f x = cross(up, z).normalize();
    Vec3f y = cross(z, x).normalize();
    Matrix Minv = Matrix::identity();
    Matrix Tr = Matrix::identity();
    for (int i = 0; i < 3; i++) {
        Minv[0][i] = x[i];
        Minv[1][i] = y[i];
        Minv[2][i] = z[i];
        Tr[i][3] = -eye[i];
    }

    ModelView = Minv * Tr;
}

//����������ƽ���ϵ�����һ�㶼��������������ε����������ʾ�����ص�����������������ı���ϵ����������P�Ϳ�����A,B,C��ʾ����������ϵ��
//���һ�������������ڣ���ô����������ϵ�����ô���0
//barycentric���þ�������������ϵ��
Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    Vec3f s[2];
    for (int i = 2; i--; ) {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    //if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
    //return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}


void triangle(mat<4, 3, float>& clipc, IShader& shader, unsigned char* framebuffer, float* zbuffer, Model* model) {
    mat<3, 4, float> pts = (Viewport * clipc).transpose(); // �����ڱ�׼�ü����ڵĶ���ӳ�䵽[x,x+w]*[y,y+h]*[0,d]�ڣ�����ת����Ϊ�˷������������һ�δ������һ����������Ƭ��������
    mat<3, 2, float> pts2;
    for (int i = 0; i < 3; i++) pts2[i] = proj<2>(pts[i] / pts[i][3]);//����wȻ��ת�ɶ�ά


    //��Χ�壬bboxminΪʲô��ʼ��Ϊ��󸡵�������Ϊ��Ҫȡ����������С�ĵ㣬����֪����С�ĵ㷶Χ��Ҫ��min���������Ծͳ�ʼ��Ϊ���ֵ��bboxmaxͬ��
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts2[i][j]));//�õ���������С�ĵ㣬��������㲻�ܱ�(0,0)��Ҳ������Ļ���½�С
            bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts2[i][j]));//ͬ���õ����ĵ㣬���ǲ��ܱ���Ļ���ϽǴ�
        }
    }
    Vec2i P;
    TGAColor color;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f bc_screen = barycentric(pts2[0], pts2[1], pts2[2], P);//��������ϵϵ��

            //͸�Ӳ�ֵ����
            Vec3f bc_clip = Vec3f(bc_screen.x / pts[0][3], bc_screen.y / pts[1][3], bc_screen.z / pts[2][3]);
            bc_clip = bc_clip / (bc_clip.x + bc_clip.y + bc_clip.z);

            float frag_depth = clipc[2] * bc_clip;//����֮��ó���ֵ�����

            //�����������ڻ��߱��ڵ��Ĳ�������Ȼ�����ڵ�����ָȫ���ڵ����������ѱ����ĵ���ڵ�����Ҳ����˵���������ĵ㱻֮ǰ�����ĵ㵲ס�Ͳ���
            //�����������ǻ��ظ���һЩ��,���Ǳ�֤������ɫ�ĵ�һ���ǵ�ǰ��ϵĵ�
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0 || zbuffer[P.x + P.y * WINDOW_WIDTH] < frag_depth) continue;
            bool discard = shader.fragment(bc_clip, color, model);//�õ���ɫ
            if (!discard) {
                zbuffer[P.x + P.y * WINDOW_WIDTH] = frag_depth;
                set_color(framebuffer, P.x, P.y, color);
            }
        }
    }
}




IShader::~IShader() {}


    


    //��д�麯��
    //������ɫ���õ��ĵ�һ��������OBJ�ļ��������ţ����а�������������Ƭ������������������������������Ͷ������� �ڶ��������ǵڼ�������
    //���ص���ͶӰ�任����οռ��еĶ�������
    Vec4f PhongShader::vertex(int index,int vert) {


        varying_uv.set_col(vert,uv[index]);//���������ζ�Ӧ��Ϊ��Ӧ����������

        //proj<3>����3��ʾ��ά��embed<4>������οռ�
        //Projection*ModelView).invert_transpose() �任�������
        //embed<4>(model->normal(iface, nthvert), 0.f���ڷ��������һά��0
        //����ķ������任֮��һ������ֱ��Ҫ���Ա任���������У��������
        varying_nrm.set_col(vert,normal[index]);
        //�Ѷ���任���������ϵ����Ա任����
        world_vert.set_col(vert, world_vert[index]);
        Vec4f gl_Vertex = clipcor[index];
        //���������ε��������㣬����������洢��һ���������еľ�����
        varying_tri.set_col(vert, gl_Vertex);
        //����οռ������任����ά����ϵ�����ҽضϵ���ά
        ndc_tri.set_col(vert, proj<3>(gl_Vertex / gl_Vertex[3]));

        
        return gl_Vertex;
    }

    //��һ�������ǽ����������������ϵ�����ڶ�����������������ɫ
    bool PhongShader::fragment(Vec3f bar, TGAColor& color, Model* model) {
        Vec3f bn = (varying_nrm * bar).normalize();
        Vec2f uv = varying_uv * bar;//��������ϵ����ϵ����ֵ��������
        Vec3f wcor = world_vert * bar;

         //����TBN������ο���ssloy/tinyrenderer TBN�����ʵ��
        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis:-tangent-space-normal-mapping
        //OPENGL��������ͼ�̳̰�TBN����ŵ�������ɫ������ƬԪ��ɫ��ʵ�֣����Խ�ʡ���㿪��,����Ч��Ҳ��̫���ˣ����Ի��Ƿ���ƬԪ��ɫ��
        mat<3, 3, float> A;
        A[0] = ndc_tri.col(1) - ndc_tri.col(0);
        A[1] = ndc_tri.col(2) - ndc_tri.col(0);
        A[2] = bn;

        mat<3, 3, float> AI = A.invert();
        Vec3f i = AI * Vec3f(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
        Vec3f j = AI * Vec3f(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);

        mat<3, 3, float> B;
        //˹����������λ��
        i = (i - i * bn * bn).normalize();
        j = (j-j*bn*bn-j*i*i).normalize();
        //TBN����
        B.set_col(0, i);
        B.set_col(1, j);
        B.set_col(2, bn);
                
        //������Ϊlg��ȫ�ֱ������ı��˾ͻ�Ӱ����һ�μ��㣬������������һ������
        //light = lg;
        light = Vec3f(-1, -1, -1).normalize();
        Vec3f n = (B * model->normal(uv)).normalize();//�ѷ����������߿ռ�任������ռ�
        Vec3f v = (eye - wcor).normalize();
        Vec3f h = (light + v).normalize();//Ϊ�˱���ÿ�ζ�Ҫ���㷴��������߼нǣ�ʹ�ð�ǽ��ƣ���n��h�нǣ�Blinn-Phongģ��

        float spec = pow(std::max(n * h, 0.0f),  model->specular(uv));//(std::pow(h*n, 5*model->specular(uv)),0.0f);
       
        float diff = std::max(0.f, n * light.normalize());//������������ɫ
        TGAColor c = model->diffuse(uv);
       
        for (int i = 0; i < 3; i++)color[i]=std::min<int>(5+c.bgra[i] , 255);
        return false;
    }


