//
//	Dept. Software Convergence, Kyung Hee University
//	Prof. Daeho Lee, nize@khu.ac.kr
//
#include "KhuGleWin.h"
#include <iostream>
#include <fstream>
//#include <cmath>
#define NOMINMAX
#include <string.h>

#pragma warning(disable:4996)

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

#ifdef _DEBUG
#ifndef DBG_NEW
#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
#define new DBG_NEW
#endif
#endif  // _DEBUG

struct Material {
	int R;
	int G;
	int B;
};

struct CKgTriangle{
	CKgVector3D v0, v1, v2;

	CKgTriangle() : v0(CKgVector3D()), v1(CKgVector3D()), v2(CKgVector3D()) {};
	CKgTriangle(CKgVector3D vv0, CKgVector3D vv1, CKgVector3D vv2)
		: v0(vv0), v1(vv1), v2(vv2) {};
}; 

class CKhuGle3DSprite : public CKhuGleSprite {
public:
	//Get vertex
	std::vector<CKgVector3D> Vertex;
	
	//Meterial을 지정
	std::vector<Material> Mtl;

	//DepthMap
	double** DepthMap;
	int DepthMapH, DepthMapW;

	std::vector<CKgTriangle> SurfaceMesh;
	std::vector<KgColor24> SurfaceMtl;

	double **m_ProjectionMatrix;
	CKgVector3D m_CameraPos;

	CKhuGle3DSprite(int nW, int nH, double Fov, double Far, double Near, KgColor24 fgColor);
	~CKhuGle3DSprite();

	static void DrawTriangle(unsigned char **R, unsigned char **G, unsigned char **B, int nW, int nH, 
		int x0, int y0, int x1, int y1, int x2, int y2, KgColor24 Color24,
		float depth1, float depth2, float depth3, double** depthmap);
	static void MatrixVector44(CKgVector3D &out, CKgVector3D v, double **M);
	static double **ComputeViewMatrix(CKgVector3D Camera, CKgVector3D Target, CKgVector3D CameraUp);

	void Render();
	void MoveBy(double OffsetX, double OffsetY, double OffsetZ);
	void CameraMove(double OffsetX, double OffsetY, double OffsetZ);

	//Open and read obj
	void ReadObj();

	float GetDepth(int y, int x);
	void NewDepth(int y, int x, float depth);
};

CKhuGle3DSprite::CKhuGle3DSprite(int nW, int nH, double Fov, double Far, double Near, KgColor24 fgColor) {
	m_fgColor = fgColor;
	m_CameraPos = CKgVector3D(0., -50, -80);

	m_ProjectionMatrix = dmatrix(4, 4);
	for(int r = 0 ; r < 4 ; ++r)
		for(int c = 0 ; c < 4 ; ++c)
			m_ProjectionMatrix[r][c] = 0.;

	//initiate DepthMap
	DepthMap = dmatrix(nH, nW);
	DepthMapH = nH;
	DepthMapW = nW;

	//클수록 멀리, 작을수록 가까이
	for (int i = 0; i < nH; i++)
		for (int j = 0; j < nW; j++)
			DepthMap[i][j] = 10000;

	m_ProjectionMatrix[0][0] = (double)nH/(double)nW * 1./tan(Fov/2.);
	m_ProjectionMatrix[1][1] = 1./tan(Fov/2.);
	m_ProjectionMatrix[2][2] = (-Near-Far) / (Near-Far);
	m_ProjectionMatrix[2][3] = 2.*(Far * Near) / (Near-Far);
	m_ProjectionMatrix[3][2] = 1.;
	m_ProjectionMatrix[3][3] = 0.;	
	
	//Read Mesh and make Surface
	ReadObj();
	/*
	SurfaceMesh.push_back(CKgTriangle(CKgVector3D(-0.5, 0., -sqrt(3.)/6), CKgVector3D(0.5, 0., -sqrt(3.)/6), CKgVector3D(0., 0., sqrt(3.)/3)));
	SurfaceMesh.push_back(CKgTriangle(CKgVector3D(0., 0., sqrt(3.)/3), CKgVector3D(0.5, 0., -sqrt(3.)/6), CKgVector3D(0., sqrt(3.)/3, 0.)));
	SurfaceMesh.push_back(CKgTriangle(CKgVector3D(-0.5, 0., -sqrt(3.)/6), CKgVector3D(0, 0., sqrt(3.)/3), CKgVector3D(0., sqrt(3.)/3, 0.)));
	SurfaceMesh.push_back(CKgTriangle(CKgVector3D(0.5, 0., -sqrt(3.)/6), CKgVector3D(-0.5, 0., -sqrt(3.)/6), CKgVector3D(0., sqrt(3.)/3, 0.)));																						   
	*/
};

CKhuGle3DSprite::~CKhuGle3DSprite() {
	
	free_dmatrix(DepthMap, DepthMapH, DepthMapW);

	free_dmatrix(m_ProjectionMatrix, 4, 4);
};

void CKhuGle3DSprite::ReadObj()
{
	//Mtl 지정
	//Kd = color

	//파일의 첫 단어를 읽어온다.
	char lineHeader[120];

	//읽어 온 데이터의 개수를 파악하기 위함(face에서 쓰임)
	int count = 0;

	//material을 읽어 와서 MTL에 저장
	FILE* fpp;
	fpp = fopen("car.mtl", "r");

	float R1, G1, B1;
	Material mt;
	while (1)
	{
		int res = fscanf(fpp, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. 파일이 끝나면 while문도 끝

		//kd가 material의 기본 색상
		if (strcmp(lineHeader, "Kd") == 0)
		{
			count = fscanf(fpp, "%f %f %f /n", &R1, &G1, &B1);
			//소수로 입력받은 RGB 비율을, KgColor24에 맞추기 위해 0~255로 변경
			mt.R = (int)(R1 * 255);
			mt.G = (int)(G1 * 255);
			mt.B = (int)(B1 * 255);
			Mtl.push_back(mt);
		}
	}

	fclose(fpp);

	FILE* fp;
	fp = fopen("car.obj", "r");

	float x, y, z;
	int v1, v2, v3, v4, vt1, vt2, vt3, vt4, vn1, vn2, vn3, vn4;

	int mtl_index = 0;

	float miny = 100;

	while(1)
	{
		int res = fscanf(fp, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. 파일이 끝나면 while문도 끝

		//첫 단어에 따라 vertex인지 face인지, MTL인지 확인.
		if (strcmp(lineHeader, "v") == 0)
		{
			count = fscanf(fp, "%f %f %f /n", &x, &y, &z);
			CKgVector3D v(x * 7, y * -7, z * 7);
			if (miny > (y * (-7)))
				miny = y * (-7);
			Vertex.push_back(v);
		}
		else if (strcmp(lineHeader, "f") == 0)
		{
			count = fscanf(fp, "%d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d /n", &v1, &vt1, &vn1, &v2, &vt2, &vn2, &v3, &vt3, &vn3, &v4, &vt4, &vn4);
			//face가 삼각인지 사각인지 확인
			//삼각형이면 정상적으로 Surface에 push
			if (count == 9)
			{
				CKgTriangle t(Vertex[v1 - 1], Vertex[v2 - 1], Vertex[v3 - 1]);
				SurfaceMtl.push_back(KG_COLOR_24_RGB(Mtl[mtl_index].R, Mtl[mtl_index].G, Mtl[mtl_index].B));
				SurfaceMesh.push_back(t);
			}
			//사각형이면 삼각형으로 분할해서 push
			//[1,2,3] [3,4,1]
			else if (count == 12)
			{
				CKgTriangle t(Vertex[v1 - 1], Vertex[v2 - 1], Vertex[v3 - 1]);
				CKgTriangle t2(Vertex[v3 - 1], Vertex[v4 - 1], Vertex[v1 - 1]);
				SurfaceMtl.push_back(KG_COLOR_24_RGB(Mtl[mtl_index].R, Mtl[mtl_index].G, Mtl[mtl_index].B));
				SurfaceMtl.push_back(KG_COLOR_24_RGB(Mtl[mtl_index].R, Mtl[mtl_index].G, Mtl[mtl_index].B));
				SurfaceMesh.push_back(t);
				SurfaceMesh.push_back(t2);
			}
		}
		else if (strcmp(lineHeader, "usemtl") == 0)
		{
			char mtlReader[15];
			int temp = fscanf(fp, " %s", mtlReader);
			//11번째(0부터 시작시) 즉 12개의 문자
			//처음 Material은 이름이 Material이므로 패스. 이미 0번 인덱스 지정됨.
			if (strlen(mtlReader) > 10)
			{
				mtl_index = mtlReader[11] - '0';
			}
		}
	}
	std::cout << miny;
	
	CKgVector3D f1(20, 30, 20), f2(20, 30, -20), f3(-20, 30, 20);
	CKgTriangle floor(f1, f2, f3);
	CKgVector3D ff1(-20, 30, -20), ff2(-20, 30, 20), ff3(20, 30,-20);
	CKgTriangle ffloor(ff1, ff2, ff3);
	SurfaceMtl.push_back(KG_COLOR_24_RGB(255, 255, 255));
	SurfaceMesh.push_back(floor);
	SurfaceMtl.push_back(KG_COLOR_24_RGB(255, 255, 255));
	SurfaceMesh.push_back(ffloor);
	
	fclose(fp);
}


float CKhuGle3DSprite::GetDepth(int y, int x)
{
	return DepthMap[y][x];
}

void CKhuGle3DSprite::NewDepth(int y, int x, float depth)
{
	DepthMap[y][x] = depth;
}

void CKhuGle3DSprite::DrawTriangle(unsigned char **R, unsigned char **G, unsigned char **B, int nW, int nH, 
	int x0, int y0, int x1, int y1, int x2, int y2, KgColor24 Color24,
	float depth1, float depth2, float depth3, double** depthmap)
{
	/*
	//삼각형을 그리기에 앞서, 높이 순으로 정렬
	int xx0, xx1, xx2, xx3;
	int yy0, yy1, yy2, yy3;
	//yy2 > yy1 > yy0
	if (y2 > y1)
	{
		if (y1 > y0)
		{
			yy2 = y2;			xx2 = x2;
			yy1 = y1;			xx1 = x1;
			yy0 = y0;			xx0 = x0;
		}
		else
		{
			if (y2 > y0)
			{
				//2 0 1
				yy2 = y2;			xx2 = x2;
				yy1 = y0;			xx1 = x0;
				yy0 = y1;			xx0 = x1;
			}
			else
			{
				//0 2 1
				yy2 = y0;			xx2 = x0;
				yy1 = y2;			xx1 = x2;
				yy0 = y1;			xx0 = x1;
			}
		}
	}
	else //y1 > y2
	{
		if (y2 > y0)
		{
			//1 2 0
			yy2 = y1;			xx2 = x1;
			yy1 = y2;			xx1 = x2;
			yy0 = y0;			xx0 = x0;
		}
		else // y1 > y2 y0 > y2
		{
			if (y1 > y0) // 1 0 2
			{
				yy2 = y1;			xx2 = x1;
				yy1 = y0;			xx1 = x0;
				yy0 = y2;			xx0 = x2;
			}
			else
			{
				yy2 = y0;			xx2 = x0;
				yy1 = y1;			xx1 = x1;
				yy0 = y2;			xx0 = x2;
			}
		}
	}

	//Fill Bottom Flat triangle
	float invslope1, invslope2, curx1, curx2;

	if (yy2 == yy1)
	{
		if (xx1 - xx0 == 0)	invslope1 = 0;
		else invslope1 = (xx1 - xx0) / (xx1 - xx0);

		if (xx2 - xx0 == 0)	invslope2 = 0;
		else invslope2 = (xx2 - xx0) / (xx2 - xx0);

		curx1 = xx0;
		curx2 = xx0;

		for (int scanlineY = yy0; scanlineY <= yy1; scanlineY++)
		{
			CKhuGleSprite::DrawLine(R, G, B, nW, nH, (int)curx1, scanlineY, (int)curx2, scanlineY, Color24);
			curx1 -= invslope1;
			curx2 -= invslope2;
		}
	}
	else if (yy0 == yy1) //top flat triangle
	{
		if (xx2 - xx0 == 0)	invslope1 = 0;
		else invslope1 = (xx2 - xx0) / (xx2 - xx0);

		if (xx2 - xx1 == 0)	invslope2 = 0;
		else invslope2 = (xx2 - xx1) / (xx2 - xx1);

		curx1 = xx2;
		curx2 = xx2;

		for (int scanlineY = yy2; scanlineY > yy0; scanlineY--)
		{
			CKhuGleSprite::DrawLine(R, G, B, nW, nH, (int)curx1, scanlineY, (int)curx2, scanlineY, Color24);
			curx1 -= invslope1;
			curx2 -= invslope2;
		}
	}
	else
	{
		xx3 = (int)(xx0 + ((float)(yy1 - yy0) / (float)(yy2 - yy0)) * (xx2 - xx0));
		yy3 = yy2;

		//fill bottom split triangle
		if (xx1 - xx0 == 0)	invslope1 = 0;
		else invslope1 = (xx1 - xx0) / (xx1 - xx0);
		if (xx3 - xx0 == 0)	invslope2 = 0;
		else invslope2 = (xx3 - xx0) / (xx3 - xx0);

		curx1 = xx0;
		curx2 = xx0;

		for (int scanlineY = yy0; scanlineY <= yy1; scanlineY++)
		{
			CKhuGleSprite::DrawLine(R, G, B, nW, nH, (int)curx1, scanlineY, (int)curx2, scanlineY, Color24);
			curx1 -= invslope1;
			curx2 -= invslope2;
		}

		//fill top split triangle
		if (xx2 - xx1 == 0)	invslope1 = 0;
		else invslope1 = (xx2 - xx1) / (xx2 - xx1);
		if (xx2 - xx3 == 0)	invslope2 = 0;
		else invslope2 = (xx2 - xx3) / (xx2 - xx3);

		curx1 = xx2;
		curx2 = xx2;

		for (int scanlineY = yy2; scanlineY > yy0; scanlineY--)
		{
			CKhuGleSprite::DrawLine(R, G, B, nW, nH, (int)curx1, scanlineY, (int)curx2, scanlineY, Color24);
			curx1 -= invslope1;
			curx2 -= invslope2;
		}
	}
	*/
	
	//Barycentric Algorithm
	//삼각형에 대한 바운딩 박스를 정의
	int maxX = max(x0, max(x1, x2));
	int minX = min(x0, min(x1, x2));
	int maxY = max(y0, max(y1, y2));
	int minY = min(y0, min(y1, y2));

	//두 개의 벡터를 얻은 후 외적을 통해 삼각형 내부에 있는지 판단
	int vs1x = x1 - x0;
	int vs1y = y1 - y0;
	int vs2x = x2 - x0;
	int vs2y = y2 - y0;

	for (int x = minX; x <= maxX; x++)
	{
		for (int y = minY; y <= maxY; y++)
		{
			int qx = x - x0;
			int qy = y - y0;

			float s = (float)((qx * vs2y) - (qy * vs2x)) / (float)((vs1x * vs2y) - (vs1y * vs2x));
			float t = (float)((vs1x * qy) - (vs1y * qx)) / (float)((vs1x * vs2y) - (vs1y * vs2x));

			if ((s >= 0) && (t >= 0) && (s + t <= 1))
			{ /* inside triangle */
				//범위 내의 pixel만 구현
				if (x > 10 && x < nW - 10 && y > 10 && y < nH-10)
				{
					//Check DepthMap
					if (depthmap[y][x] > depth1 + depth2 + depth3)
					{
						depthmap[y][x] = depth1 + depth2 + depth3;
						R[y][x] = KgGetRed(Color24);
						G[y][x] = KgGetGreen(Color24);
						B[y][x] = KgGetBlue(Color24);
					}
				}
			}
		}
	}


	//CKhuGleSprite::DrawLine(R, G, B, nW, nH, x0, y0, x1, y1, Color24);
	//CKhuGleSprite::DrawLine(R, G, B, nW, nH, x1, y1, x2, y2, Color24);
	//CKhuGleSprite::DrawLine(R, G, B, nW, nH, x2, y2, x0, y0, Color24);

}

void CKhuGle3DSprite::MatrixVector44(CKgVector3D &out, CKgVector3D v, double **M)
{
	out.x = v.x*M[0][0] + v.y*M[0][1] + v.z*M[0][2] + M[0][3];
	out.y = v.x*M[1][0] + v.y*M[1][1] + v.z*M[1][2] + M[1][3];
	out.z = v.x*M[2][0] + v.y*M[2][1] + v.z*M[2][2] + M[2][3];

	double w = v.x*M[3][0] + v.y*M[3][1] + v.z*M[3][2] + M[3][3];

	if(fabs(w) > 0)
		out = (1./w)*out;
}

double **CKhuGle3DSprite::ComputeViewMatrix(CKgVector3D Camera, CKgVector3D Target, CKgVector3D CameraUp)
{
	CKgVector3D Forward = Target-Camera;
	Forward.Normalize();
	CameraUp.Normalize();
	CKgVector3D Right = CameraUp.Cross(Forward); 
	CKgVector3D Up = Forward.Cross(Right);

	double **RT = dmatrix(4, 4);
	double **View = dmatrix(4, 4);	

	RT[0][0] = Right.x;		RT[1][0] = Right.y;		RT[2][0] = Right.z;		RT[3][0] = 0.;
	RT[0][1] = Up.x;		RT[1][1] = Up.y;		RT[2][1] = Up.z;		RT[3][1] = 0.;
	RT[0][2] = Forward.x;	RT[1][2] = Forward.y;	RT[2][2] = Forward.z;	RT[3][2] = 0.;
	RT[0][3] = Camera.x;	RT[1][3] = Camera.y;	RT[2][3] = Camera.z;	RT[3][3] = 1.;

	bool bInverse = InverseMatrix(RT, View, 4);
	
	free_dmatrix(RT, 4, 4);

	if(bInverse)
		return View;

	return nullptr;

}

void CKhuGle3DSprite::Render()
{
	if(!m_Parent) return;
	
	for (int i = 0; i < DepthMapH; i++)
		for (int j = 0; j < DepthMapW; j++)
			DepthMap[i][j] = 10000;

	/*
	double NewX = m_CameraPos.x*cos(Pi/1000.) - m_CameraPos.z*sin(Pi/1000.);
	double NewZ = m_CameraPos.x*sin(Pi/1000.) + m_CameraPos.z*cos(Pi/1000.);
	m_CameraPos.x = NewX;
	m_CameraPos.z = NewZ;
	*/

	CKhuGleLayer *Parent = (CKhuGleLayer *)m_Parent;

	//Camera , Target , Up
	double **ViewMatrix = ComputeViewMatrix(m_CameraPos, CKgVector3D(0., 0., 0.), CKgVector3D(0., 1., 0.));

	if(ViewMatrix == nullptr) return;

	int i = 0;
	for(auto &Triangle: SurfaceMesh)
	{
		CKgVector3D Side01, Side02, Normal;

		Side01 = Triangle.v1 - Triangle.v0;
		Side02 = Triangle.v2 - Triangle.v0;

		Normal = Side01.Cross(Side02);
		Normal.Normalize();

		CKgTriangle ViewTriangle;
		CKgTriangle Projected;
		
		//카메라에 보이는 위치라면!
		//if(Normal.Dot(Triangle.v0 - m_CameraPos) < 0.)
		{
			//Depth를 생성
			//ViewMatrix 상에서의 깊이를 기록하고 넘겨준다.
			float DepthV0, DepthV1, DepthV2;
			MatrixVector44(ViewTriangle.v0, Triangle.v0, ViewMatrix);
			MatrixVector44(ViewTriangle.v1, Triangle.v1, ViewMatrix);
			MatrixVector44(ViewTriangle.v2, Triangle.v2, ViewMatrix);

			DepthV0 = ViewTriangle.v0.z;
			DepthV1 = ViewTriangle.v1.z;
			DepthV2 = ViewTriangle.v2.z;
					
			MatrixVector44(Projected.v0, ViewTriangle.v0, m_ProjectionMatrix);
			MatrixVector44(Projected.v1, ViewTriangle.v1, m_ProjectionMatrix);
			MatrixVector44(Projected.v2, ViewTriangle.v2, m_ProjectionMatrix);
			
			Projected.v0.x += 1.; 
			Projected.v0.y += 1.;
			Projected.v1.x += 1.; 
			Projected.v1.y += 1.;
			Projected.v2.x += 1.; 
			Projected.v2.y += 1.;
			Projected.v0.x *= Parent->m_nW/2.;
			Projected.v0.y *= Parent->m_nH/2.;
			Projected.v1.x *= Parent->m_nW/2.;
			Projected.v1.y *= Parent->m_nH/2.;
			Projected.v2.x *= Parent->m_nW/2.;
			Projected.v2.y *= Parent->m_nH/2.;
			Projected.v0.x -= 1.; 
			Projected.v0.y -= 1.;
			Projected.v1.x -= 1.; 
			Projected.v1.y -= 1.;
			Projected.v2.x -= 1.; 
			Projected.v2.y -= 1.;
			
			//이곳에서 Color를 보냄
			m_fgColor = SurfaceMtl[i];
			DrawTriangle(Parent->m_ImageR, Parent->m_ImageG, Parent->m_ImageB, 
				Parent->m_nW, Parent->m_nH, 
				(int)Projected.v0.x, (int)Projected.v0.y, 
				(int)Projected.v1.x, (int)Projected.v1.y, 
				(int)Projected.v2.x, (int)Projected.v2.y, m_fgColor,
				DepthV0, DepthV1, DepthV2, DepthMap);
		}
		++i;
	}

	free_dmatrix(ViewMatrix, 4, 4);
}

void CKhuGle3DSprite::MoveBy(double OffsetX, double OffsetY, double OffsetZ)
{
	int i = 0;
	for(auto &Triangle: SurfaceMesh)
	{
		++i;
		Triangle.v0 = Triangle.v0 + CKgVector3D(OffsetX, OffsetY, OffsetZ);
		Triangle.v1 = Triangle.v1 + CKgVector3D(OffsetX, OffsetY, OffsetZ);
		Triangle.v2 = Triangle.v2 + CKgVector3D(OffsetX, OffsetY, OffsetZ);
		if (i == 5284)
			break;
	}
}

void CKhuGle3DSprite::CameraMove(double OffsetX, double OffsetY, double OffsetZ)
{
	m_CameraPos.x += OffsetX;
	m_CameraPos.y += OffsetY;
	m_CameraPos.y += OffsetZ;
}


class CThreeDim : public CKhuGleWin
{
public:
	CKhuGleLayer *m_pGameLayer;

	CKhuGle3DSprite *m_pObject3D;
	
	CThreeDim(int nW, int nH);
	void Update();

	CKgPoint m_LButtonStart, m_LButtonEnd;
	int m_nLButtonStatus;
};

CThreeDim::CThreeDim(int nW, int nH) : CKhuGleWin(nW, nH) 
{
	m_nLButtonStatus = 0;

	m_Gravity = CKgVector2D(0., 98.);
	m_AirResistance = CKgVector2D(0.1, 0.1);

	m_pScene = new CKhuGleScene(640, 480, KG_COLOR_24_RGB(100, 100, 150));

	m_pGameLayer = new CKhuGleLayer(600, 420, KG_COLOR_24_RGB(150, 150, 200), CKgPoint(20, 30));
	m_pScene->AddChild(m_pGameLayer);

	m_pObject3D = new CKhuGle3DSprite(m_pGameLayer->m_nW, m_pGameLayer->m_nH, Pi/2., 1000., 0.1, KG_COLOR_24_RGB(255, 255, 255));

	m_pGameLayer->AddChild(m_pObject3D);
}

void CThreeDim::Update()
{
	if(m_bKeyPressed[VK_DOWN]) 
		m_pObject3D->CameraMove(0, 0.1, 0);
	if (m_bKeyPressed[VK_UP])
		m_pObject3D->CameraMove(0, -0.1, 0);
	if (m_bKeyPressed[VK_RIGHT])
		m_pObject3D->CameraMove(0.1, 0, 0);
	if (m_bKeyPressed[VK_LEFT])
		m_pObject3D->CameraMove(-0.1, 0, 0);
	if (m_bKeyPressed['W'])
		m_pObject3D->MoveBy(0, 0, 0.1);
	if (m_bKeyPressed['S'])
		m_pObject3D->MoveBy(0, 0, -0.1);
	if (m_bKeyPressed['A'])
		m_pObject3D->MoveBy(-0.1, 0, 0);
	if (m_bKeyPressed['D'])
		m_pObject3D->MoveBy(0.1, 0, 0);

	m_pScene->Render();
	DrawSceneTextPos("3D Rendering", CKgPoint(0, 0));

	CKhuGleWin::Update();
}

int main()
{
	CThreeDim *pThreeDim = new CThreeDim(640, 480);

	KhuGleWinInit(pThreeDim);

	return 0;
}