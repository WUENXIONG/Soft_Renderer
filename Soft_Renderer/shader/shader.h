#pragma once
#include "../tool/tgaimage.h"
#include "../tool/geometry.h"

//���������������һ��shader�Ļ���ͷ�ļ��������涨���ڽṹ����ĺ������б�����ʲô�����ء����Կ�����ȫ�ֵ�һЩ�����ͺ���������shader�ļ���ʹ�ã�������
//������shaderͷ�ļ��ĵط�Ҳ����ֱ��ʹ��

extern Matrix ModelView;//������������Ϊ��ȫ�ֵģ����Կ���ֱ��������ĺ������б���ֵ
extern Matrix Projection;
//���ڵĿ�ߣ���triangleʵ����ᱻ�õ�
extern const int WINDOW_HEIGHT;
extern const int WINDOW_WIDTH;


void viewport(int x, int y, int w, int h); //ModelView����
void projection(float fovy, float aspect, float near, float far);//ͶӰ���� 
void lookat(Vec3f eye, Vec3f center, Vec3f up); //�ӿڱ任����
void set_color(unsigned char* framebuffer, int x, int y, TGAColor c);//����framebufferĳһ����ɫֵ������triangle�е���

void clip_prepared(int iface, int nthvert, Model* model);
int clipping();

//�����IShader����
struct IShader {
    mat<2, 3, float> varying_uv;  // �����ε��������꣬�ɶ�����ɫ����ֵ������ΪƬԪ��ɫ��������
    mat<3, 3, float> world_vert; //�����ζ������������
    mat<4, 3, float> varying_tri; // �����ε��������㣬����Ķ������ڲü��ռ����
    mat<3, 3, float> varying_nrm; // ��������ķ������������ķ������Ѿ���У������
    mat<3, 3, float> ndc_tri;     // ���������εĶ����Ѿ��ڱ�׼�������Ӽ�����
    virtual ~IShader();
    virtual Vec4f vertex(int index, int vert) = 0;//���麯������������Ǻ���ģ���Ϊ���е�shader������ʵ�ֶ�����ɫ����ƬԴ��ɫ��
    virtual bool fragment(Vec3f bar, TGAColor &color, Model* model) = 0;
};

struct PhongShader : public IShader {

   
    virtual Vec4f vertex(int index, int vert);
    virtual bool fragment(Vec3f bar, TGAColor& color, Model* mdoel);
};

void triangle(mat<4, 3, float>& clipc, IShader& shader, unsigned char* framebuffer, float* zbuffer, Model* model);


