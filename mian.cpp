
#include <stdlib.h>
#include <stdio.h>
#include "OSSocket.h"
#include "JsonParse.h"
#include "CmdParse.h"
#include <vector>
#include <list>
#include <math.h>
#include <map>
#include <algorithm>

#define MAX_SOCKET_BUFFER       (1024 * 1024 * 4)       /// ���ͽ����������4M
#define UAVNUM_EACHLINE 4                              //��ͼ��ÿһ����ͣ�ŵĿ��зɻ�����
#define FREEUAVPATHBUFF 1000                           //���зɻ�����·���ĺ��沿�ֳ���
#define STARTFINDINGGOODS_TIME    10                   //�ɻ����յ��ȴ��˻�ʱ��
#define UAVTOGOODSSTEP_UP         100                  //�ɻ�������Ĳ��������ɳ���ֵ
#define UAVNUMTOTRANSPORTLOWVALUE 15                   //���ɻ������������ʱ����ȥ����С��ֵ����
#define RATIOOFGOODSVALUE         2                    //ǰ�ڷɻ����͵Ļ������ͼ�ֵϵ��
#define NUMOFGOODSFORUAV          2                    //��ÿ�ҷɻ�����Ļ�����
#define MINNUMOFUAVTOFIGHT        14                   //ǰ����ս�ķɻ�����
#define MINNUMOFUAVWORKING        50                   //���зɻ��������ֵ
#define UAVDELAYOUTTIME           40                   //Ԥ���ķɻ������������ӳٵ�ʱ��
using namespace std;

const int kCost1 = 10; //ֱ��һ������
const int kCost2 = 10; //б��һ������

struct Point
{
	int x, y; //�����꣬����Ϊ�˷��㰴��C++�����������㣬x������ţ�y��������
	int F, G, H; //F=G+H
	Point *parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴���
	Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //������ʼ��
	{
	}
};
struct Point3d
{
	int x, y, z; //�����꣬����Ϊ�˷��㰴��C++�����������㣬x������ţ�y��������

	Point3d(int _x, int _y, int _z) :x(_x), y(_y), z(_z) //������ʼ��
	{
	}
	Point3d() {}
};

struct Uav
{
	int num;//�ɻ����
	bool isfree;//�Ƿ�ɻ��ǿ���״̬��
	bool issetpath;//�Ƿ���ɻ�����·��
	int time_temp;//�ɻ�·����ʱ�����
	Point3d goodsposition;//�ɻ�Ҫ���ͻ�������
	Point3d targetposition;//�ɻ�Ҫ���ͻ�����յ�
	Point3d freetimeposition;//����ʱ��ͣ�ŵ�λ��
	Point3d fightingpoint;//���صл��ĵص�
	Point3d defendingpoint;
	Point3d defendingpoint1;
	vector<Point3d> path3d;//�ɻ��ķ���·��
	vector<Point3d> pathstarttoend;//������㵽�յ��·��
	vector<Point3d> pathtogoods;//�ɻ��������·��
	int goodstotransport;//�ɻ����ͻ���ı��
	bool isstoped;//�Ƿ�ɻ�ͣ��ԭ��
	bool isfighting;//�Ƿ�ɻ�Ҫ����ս��
	bool istranspointing;//�Ƿ������ͻ���
	int loadweight1;//���������
	int loadweight2;//���������������
	int goodsweight;//���͵Ļ��������
	int goodsvalue;//���͵Ļ���ļ�ֵ
	int goodsrelevatevalue;//�������Լ�ֵ
	int remainelectricity;//�ɻ�ʣ�����
	bool ischarging;//�ɻ��Ƿ��ڳ��
	bool isfullcharge;//�ɻ��Ƿ������
	int capacity;//�������
	int charge;//��λʱ������
	bool isdefending;//�Ƿ��ڷ���
	bool isdefending1;
	bool isdefended;
	bool isdefended1;
	int idofuavtodefend;
	int idofuavtodefend1;
	int idofdefendinguav;
	int idofdefendinguav1;
	bool isbackinghome;
	int value;
	bool isalreadybackinghome;
	int idofenemyuavtofight;
	bool istotransporting;
	Uav(int _num) :num(_num) {
		isfree = false;
		time_temp = -1;
		issetpath = false;
		isstoped = false;

		isfighting = false;

		istranspointing = false;
		isfullcharge = false;
		ischarging = true;
		isdefending = false;
		isdefended = false;
		idofdefendinguav = -1;
		idofuavtodefend = -1;

		isbackinghome = false;
		isalreadybackinghome = false;
		goodstotransport = -1;
		idofenemyuavtofight = -1;
		istotransporting = false;
		isdefending1 = false;
		isdefended1 = false;
		idofdefendinguav1 = -1;
		idofuavtodefend1 = -1;


	}
	Uav() {}
};
struct Goods
{
	int num;
	bool ischoosed;
	int value;
	int weight;
	vector<Point3d> pathstarttoend;
	int chargecost;
	Point3d pstart;
	Point3d pend;
	Goods(int _num) :num(_num) {}
	Goods() {}
};
class Astar
{
public:
	void InitAstar(std::vector<std::vector<char>> &_maze);
	std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);

private:
	Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
	std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
	bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; //�ж�ĳ���Ƿ����������һ���ж�
	Point *isInList(const std::list<Point *> &list, const Point *point) const; //�жϿ���/�ر��б����Ƿ����ĳ��
	Point *getLeastFpoint(); //�ӿ����б��з���Fֵ��С�Ľڵ�
							 //����FGHֵ
	int calcG(Point *temp_start, Point *point);
	int calcH(Point *point, Point *end);
	int calcF(Point *point);
private:
	std::vector<std::vector<char>> maze;
	std::list<Point *> openList;  //�����б�
	std::list<Point *> closeList; //�ر��б�
};
struct Gloablestatus
{
	int numofdestroyeduav;//ײ�ٵķɻ�����
	vector<Point3d> freepointsvector;//���еķɻ�����ʱͣ�ŵص�
	int maxfreepointsindex;//����ͣ�ŵ�����
	bool isgoodsweightcounted;//�Ƿ�ͳ�ƹ���ͬ�������������
	vector<int> numofgoodsateachweight;//����ÿ����������Ļ�������
	vector<int> uavquenetoperchase;//Ҫ����ɻ��Ĵ�������
	int indexofuavquenetoperchase;//Ҫ����ɻ��Ĵ����������±�
	vector<int> enemyuavchoosed;
	int heightofworkinguav;//�ɻ������߶�
	Point3d enemypoint;//�з�ͣ��ƺλ��
	map<int, Goods> goodsmap;
	map<string, UAV_PRICE> uavprice;
	vector<int> timetostopuavs1;
	vector<int> timetostopuavs2;
	bool isbackinghome;
	bool ismoneyenough;
};

void Astar::InitAstar(std::vector<std::vector<char>> &_maze)
{
	maze = _maze;
}

int Astar::calcG(Point *temp_start, Point *point)
{
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	int parentG = point->parent == NULL ? 0 : point->parent->G; //����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿ�
	return parentG + extraG;
}

int Astar::calcH(Point *point, Point *end)
{
	return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y))*kCost1;
}

int Astar::calcF(Point *point)
{
	return point->G + point->H;
}

Point *Astar::getLeastFpoint()
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto &point : openList)
			if (point->F < resPoint->F)
				resPoint = point;
		return resPoint;
	}
	return NULL;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	openList.push_back(new Point(startPoint.x, startPoint.y)); //�������,��������һ���ڵ㣬�������
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		openList.remove(curPoint); //�ӿ����б���ɾ��
		closeList.push_back(curPoint); //�ŵ��ر��б�
									   //1,�ҵ���ǰ��Χ�˸����п���ͨ���ĸ���
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ��Ϊ�丸�ڵ㣬����F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG < target->G)
				{
					target->parent = curPoint;

					target->G = tempG;
					target->F = calcF(target);
				}
			}
			Point *resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
		}
	}

	return NULL;
}

std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//����·�������û�ҵ�·�������ؿ�����
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
	return path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
	for (auto p : list)
		if (p->x == point->x&&p->y == point->y)
			return p;
	return NULL;
}

bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{
	//printf("�±�x,y��%d,%d\n", target->x, target->y);
	//int test = maze[target->x][target->y] == 1;
	if (target->x<0 || target->x>maze.size() - 1 || target->y<0 || target->y>maze[0].size() - 1) {
		return false;
	}
	else if (maze[target->x][target->y] == 1 || (target->x == point->x&&target->y == point->y)) {
		return false;
	}
	else if (isInList(closeList, target)) {
		return false;
	}
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //��б�ǿ���
			return true;
		else
		{
			//б�Խ�Ҫ�ж��Ƿ��ס
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
				return true;
			else
				return isIgnoreCorner;
		}
	}
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;

	for (int x = point->x - 1; x <= point->x + 1; x++) {
		for (int y = point->y - 1; y <= point->y + 1; y++) {
			if (isCanreach(point, new Point(x, y), isIgnoreCorner))
			{
				surroundPoints.push_back(new Point(x, y));
			}
		}
	}

	return surroundPoints;
}

/** @fn     int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)
 *  @brief	��������
 *	@param  -I   - OS_SOCKET hSocket
 *	@param  -I   - char * pBuffer
 *	@return int
 */
int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)
{
	int         nRecvLen = 0;
	int         nLen = 0;
	while (1)
	{
		// ����ͷ������
		nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
		if (nLen <= 0)
		{
			printf("recv error\n");
			return nLen;
		}
		nRecvLen += nLen;

		if (nRecvLen >= SOCKET_HEAD_LEN)
		{
			break;
		}
	}
	int         nJsonLen = 0;
	char        szLen[10] = { 0 };
	memcpy(szLen, pBuffer, SOCKET_HEAD_LEN);
	nJsonLen = atoi(szLen);
	while (nRecvLen < (SOCKET_HEAD_LEN + nJsonLen))
	{
		// ˵�����ݻ�û������
		nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
		if (nLen <= 0)
		{
			printf("recv error\n");
			return nLen;
		}
		nRecvLen += nLen;
	}
	return 0;
}

/** @fn     int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
 *  @brief	��������
 *	@param  -I   - OS_SOCKET hSocket
 *	@param  -I   - char * pBuffer
 *	@param  -I   - int nLen
 *	@return int
 **/
int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
{
	int     nSendLen = 0;
	int     nLenTmp = 0;

	while (nSendLen < nLen)
	{
		nLenTmp = OSSend(hSocket, pBuffer + nSendLen, nLen - nSendLen);
		if (nLenTmp < 0)
		{
			return -1;
		}

		nSendLen += nLenTmp;
	}

	return 0;
}

/** @fn     void AlgorithmCalculationFun()
 *  @brief	ѧ�����㷨���㣬 ����ʲô�Ķ��Լ�д��
 *	@return void
 */
void  AlgorithmCalculationFun(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<vector<vector<char>>> map3d, int time_temp, vector<Point3d> path3d);

void printinformation(MAP_INFO *pstMap)
{
	printf("��ͷ��и߶�: %d\n", pstMap->nHLow);
	printf("��߷��и߶�: %d\n", pstMap->nHHigh);
	printf("��ͼ����: %d\n", pstMap->nMapX);
	printf("��ͼ���u: %d\n", pstMap->nMapY);
	printf("��ͼ�߶�: %d\n", pstMap->nMapZ);
	printf("parkingx�� %d\n", pstMap->nParkingX);
	printf("parkingy�� %d\n", pstMap->nParkingY);
	printf("buildingnum�� %d\n", pstMap->nBuildingNum);
	printf("forgnum�� %d\n", pstMap->nFogNum);
	printf("uavnum�� %d\n", pstMap->nUavNum);
	printf("uavpricenum�� %d\n", pstMap->nUavPriceNum);
	for (int i = 0; i < pstMap->nBuildingNum; i++) {
		printf("building height: %d\n", (pstMap->astBuilding)[i].nH);
	}
	printf("�ϰ�����ʼλ�ã�%d,%d\n", pstMap->astBuilding[0].nX, pstMap->astBuilding[0].nY);
	printf("�ϰ��ﳤ�ȺͿ�Ⱥ͸߶ȣ�%d,%d,%d\n", pstMap->astBuilding[0].nL, pstMap->astBuilding[0].nW, pstMap->astBuilding[0].nH);
}
void attractmap2d(MAP_INFO *pstMap, vector<vector<vector<char>>> &map2d) {

}
void attractmap3d(MAP_INFO *pstMap, vector<vector<vector<char>>> &map3d) {
	for (size_t i = 0; i < pstMap->nMapZ; i++) {
		for (size_t j = 0; j < pstMap->nMapX; j++) {
			for (size_t k = 0; k < pstMap->nMapY; k++) {
				map3d[i][j][k] = 0;
			}
		}
	}
	for (size_t i = 0; i < pstMap->nBuildingNum; i++) {
		for (size_t j = 0; j < pstMap->astBuilding[i].nH; j++) {
			for (size_t k = 0; k < pstMap->astBuilding[i].nL; k++) {
				for (size_t w = 0; w < pstMap->astBuilding[i].nW; w++) {
					map3d[j][k + pstMap->astBuilding[i].nX][w + pstMap->astBuilding[i].nY] = 1;
				}
			}
		}
	}

}

void printmap3d(MAP_INFO *pstMap, vector<vector<vector<char>>> &map3d) {
	for (size_t i = 0; i < pstMap->nMapZ; i++) {
		for (size_t j = 0; j < pstMap->nMapX; j++) {
			for (size_t k = 0; k < pstMap->nMapY; k++) {
				printf("%d ", map3d[i][j][k]);
			}
			printf("\n");
		}
		printf("\n*************************\n");
	}
}
void Setuavfreetimeposition(vector<Uav>& uav, MAP_INFO *pstMap, vector<vector<vector<char>>>  map3d, Gloablestatus& gloablestatus) {
	int temp;
	for (int i = 0, j = 0; i < uav.size(); i++) {
		if (j < gloablestatus.freepointsvector.size()) {
			if (map3d[gloablestatus.freepointsvector[j].z][gloablestatus.freepointsvector[j].x][gloablestatus.freepointsvector[j].y] != 1) {
				uav[i].freetimeposition = gloablestatus.freepointsvector[j];
				j++;
			}
			else {
				i--;
				j++;
			}
		}
		temp = j;
	}
	gloablestatus.maxfreepointsindex = temp;
}

void Initialization(vector<Uav>& uav, MAP_INFO *pstMap, vector<vector<vector<char>>>  map3d, Gloablestatus& gloablestatus, vector<UAV_PRICE> uavprice) {
	for (int i = 0; i < pstMap->nUavNum; i++) {
		Uav temp(i);
		temp.loadweight1 = pstMap->astUav[i].nLoadWeight;
		for (int j = 0; j < uavprice.size(); j++) {
			if (temp.loadweight1 == uavprice[j].nLoadWeight) {
				temp.capacity = uavprice[j].capacity;
				temp.charge = uavprice[j].charge;
				temp.value = uavprice[j].nValue;
				if (j +1< uavprice.size()) {
					temp.loadweight2 = uavprice[j + 1].nLoadWeight;
				}
				else {
					temp.loadweight2 = 0;
				}
			}
		}
		uav.push_back(temp);
	}
	Setuavfreetimeposition(uav, pstMap, map3d, gloablestatus);
}

void Rankuavprice(MAP_INFO *pstMap, vector<int>& rankeduavprice, vector<UAV_PRICE>& uavprice,Gloablestatus& gloablestatus) {

	for (int i = 0; i < pstMap->nUavPriceNum; i++) {

		UAV_PRICE uav_price;
		uav_price = pstMap->astUavPrice[i];
		string str=pstMap->astUavPrice[i].szType;
		gloablestatus.uavprice.insert(pair<string, UAV_PRICE>(str, uav_price));
	}

	//rankeduavprice��uavprice���ǰ��۸�Ӹߵ�������
	int temp;
	for (int i = 0; i < pstMap->nUavPriceNum; i++) {
		rankeduavprice.push_back(pstMap->astUavPrice[i].nValue);
		//printf("%d ", gloablestatus.uavprice[pstMap->astUavPrice[i].szType].nValue);
	}
	//printf("\n");
	for (int i = 0; i < rankeduavprice.size() - 1; i++) {
		for (int j = i + 1; j < rankeduavprice.size(); j++) {
			if (rankeduavprice[i] < rankeduavprice[j]) {
				temp = rankeduavprice[i];
				rankeduavprice[i] = rankeduavprice[j];
				rankeduavprice[j] = temp;
			}
		}
	}
	for (int i = 0; i < rankeduavprice.size(); i++) {
		for (int j = 0; j < rankeduavprice.size(); j++) {
			if (rankeduavprice[i] == pstMap->astUavPrice[j].nValue) {
				UAV_PRICE uavpricetemp;
				uavpricetemp.nLoadWeight = pstMap->astUavPrice[j].nLoadWeight;
				uavpricetemp.nValue = pstMap->astUavPrice[j].nValue;
				strcpy(uavpricetemp.szType, pstMap->astUavPrice[j].szType);
				uavpricetemp.capacity = pstMap->astUavPrice[j].capacity;
				uavpricetemp.charge = pstMap->astUavPrice[j].charge;
				uavprice.push_back(uavpricetemp);
				break;
			}
		}
	}
}

void Creatfreepointvector(MAP_INFO *pstMap, vector<Point3d>& freepointsvector) {
	int minwidth = pstMap->nMapX > pstMap->nMapY ? pstMap->nMapY : pstMap->nMapX;
	int hlow = pstMap->nHLow;
	int hhigh = pstMap->nHHigh;
	int width = minwidth / UAVNUM_EACHLINE;
	vector<Point3d> unsortedpoints;
	for (int k = hlow; k < hhigh + 1; k++) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				Point3d temp(i*width + width / 2 + rand() % (width - 1) - width / 2, j*width + width / 2 + rand() % (width - 1) - width / 2, k);
				unsortedpoints.push_back(temp);
			}
		}
	}
	for (int i = 0; i < hhigh - hlow + 1; i++) {
		freepointsvector.push_back(unsortedpoints[0 + i * 16]);
		freepointsvector.push_back(unsortedpoints[3 + i * 16]);
		freepointsvector.push_back(unsortedpoints[15 + i * 16]);
		freepointsvector.push_back(unsortedpoints[12 + i * 16]);
		freepointsvector.push_back(unsortedpoints[1 + i * 16]);
		freepointsvector.push_back(unsortedpoints[7 + i * 16]);
		freepointsvector.push_back(unsortedpoints[14 + i * 16]);
		freepointsvector.push_back(unsortedpoints[8 + i * 16]);
		freepointsvector.push_back(unsortedpoints[2 + i * 16]);
		freepointsvector.push_back(unsortedpoints[11 + i * 16]);
		freepointsvector.push_back(unsortedpoints[13 + i * 16]);
		freepointsvector.push_back(unsortedpoints[4 + i * 16]);
		freepointsvector.push_back(unsortedpoints[5 + i * 16]);
		freepointsvector.push_back(unsortedpoints[6 + i * 16]);
		freepointsvector.push_back(unsortedpoints[10 + i * 16]);
		freepointsvector.push_back(unsortedpoints[9 + i * 16]);
	}
	for (int i = 0; i < freepointsvector.size(); i++) {
		printf("%d,%d,%d\n", freepointsvector[i].x, freepointsvector[i].y, freepointsvector[i].z);
	}
	printf("\n");
}

void Buyuavandinitial(vector<UAV_PRICE>& uavprice, vector<int> rankeduavprice, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, vector<vector<vector<char>>> map3d, FLAY_PLANE *pstFlayPlane, Gloablestatus& gloablestatus);
void Addpathtoflyplan(vector<int>& goodchoosed, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<vector<vector<char>>> map3d, vector<Uav> &uav, Gloablestatus& gloablestatus);
void Assigntarget(vector<UAV_PRICE>& uavprice,Gloablestatus& gloablestatus,MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, vector<vector<vector<char>>> map3d, vector<Goods>& goods, vector<int>& goodschoosed, vector<int> rankeduavprice);
void Judgecrashandadjustpath(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav);
void Oneuavflyeachtimestamp(vector<Uav>& uav,Gloablestatus& gloablestatus, MATCH_STATUS * pstMatch);
void Getenemyinfo(MATCH_STATUS * pstMatch, Gloablestatus& gloablestatus);
void Getgoodsinfo(MATCH_STATUS * pstMatch, Gloablestatus& gloablestatus, vector<vector<vector<char>>> map3d, MAP_INFO *pstMap);
inline void Setpath(Gloablestatus& gloablestatus,vector<vector<vector<char>>> map3d, Uav &uav, MATCH_STATUS * pstMatch, MAP_INFO *pstMap);
inline void Assignuavtogoods(vector<UAV_PRICE>& uavprice,Gloablestatus& gloablestatus,vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO* pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice);
inline void Adjustuavtogoods(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice);
inline void Assignuavtodefence(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice);
inline void Assignuavtoprotectuavtotransport(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice);
inline void Assignuavtocharge(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice);
inline vector<Point3d> Caculateclosestpath(Point3d startpoint, Point3d endpoint, vector<vector<vector<char>>> map3d, MAP_INFO *pstMap,int workingheight);
inline bool Isgoodschoosed(int goodsnum, vector<int> goodschoosed);
inline bool Isnewuavdestoryed(MATCH_STATUS * pstMatch, vector<Uav>& uav, Gloablestatus& gloablestatus);
inline void Setplan(vector<int> &goodschoosed, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<Uav>& uav,Gloablestatus& gloablestatus);
inline int Caculate3ddistance(Gloablestatus& gloablestatus,Point3d startpoint, Point3d endpoint, Point3d goodspoint, int& fasteststep,bool ifhigher);
inline void Setnewuavinfo(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, vector<vector<vector<char>>> map3d, int numofuav, Gloablestatus& gloablestatus);
inline bool Istwouavcollide(Point3d Anow, Point3d Anext, Point3d Bnow, Point3d Bnext);
inline void Adjustpathfortwouav(Uav& uavA, Uav& uavB, MATCH_STATUS * pstMatch, MAP_INFO *pstMap, vector<vector<vector<char>>> map3d);
inline int Caculateclosetflydistanceoftwopoints(Point3d A, Point3d B);
inline int Judgewhichuavtypetobuy(vector<UAV_PRICE>& uavprice, vector<int> rankeduavprice, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, Gloablestatus& gloablestatus);
inline void Assignuavtofight(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice);
inline bool Isenemyuavchoosed(int enemyuavnum, Gloablestatus& gloablestatue);
inline bool Ifquitloop(vector<int> temp);
inline Point3d Getadjacentpoints(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, Point3d point3d);
inline vector<Point3d> Getgoodsstarttoend(Point3d pstart, Point3d pend, MATCH_STATUS * pstMatch, Gloablestatus& gloablestatus, vector<vector<vector<char>>> map3d, MAP_INFO *pstMap);
int main(int argc, char *argv[])
{
#ifdef OS_WINDOWS
	// windows�£���Ҫ���г�ʼ������
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("WSAStartup failed\n");
		return false;
	}
#endif
	char        szIp[64] = { 0 };
	int         nPort = 0;
	char        szToken[128] = { 0 };
	int         nRet = 0;
	OS_SOCKET   hSocket;
	char        *pRecvBuffer = NULL;
	char        *pSendBuffer = NULL;
	int         nLen = 0;
	 //���ص���ȥ�����
	if (argc != 4)
	{
		printf("error arg num\n");
		return -1;
	}
	strcpy(szIp, argv[1]);
	nPort = atoi(argv[2]);
	strcpy(szToken, argv[3]);
	printf("server ip %s, prot %d, token %s\n", szIp, nPort, szToken);
	// ��ʼ���ӷ�����
	nRet = OSCreateSocket(szIp, (unsigned int)nPort, &hSocket);
	if (nRet != 0)
	{
		printf("connect server error\n");
		return nRet;
	}
	// ������ܷ����ڴ�
	pRecvBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
	if (pRecvBuffer == NULL)
	{
		return -1;
	}
	pSendBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
	if (pSendBuffer == NULL)
	{
		free(pRecvBuffer);
		return -1;
	}
	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);

	// ��������  ���ӳɹ���Judger�᷵��һ����Ϣ��
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}
	// json ����
	// ��ȡͷ��
	CONNECT_NOTICE  stNotice;
	nRet = ParserConnect(pRecvBuffer + SOCKET_HEAD_LEN, &stNotice);
	if (nRet != 0)
	{
		return nRet;
	}
	// ���ɱ�����ݵ�json
	TOKEN_INFO  stToken;
	strcpy(stToken.szToken, szToken);  // ����ǵ��Խ׶Σ�����������Ե�token�����ҵĶ�ս�л�ȡ��
								   // ʵ�ʱ�������Ҫ������Եģ�����demoд�ģ��з��������ô��롣
	strcpy(stToken.szAction, "sendtoken");

	memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = CreateTokenInfo(&stToken, pSendBuffer, &nLen);
	if (nRet != 0)
	{
		return nRet;
	}
	// ѡ������з������������(Player -> Judger)
	nRet = SendJuderData(hSocket, pSendBuffer, nLen);
	if (nRet != 0)
	{
		return nRet;
	}
	//�����֤���(Judger -> Player)��
	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}
	// ������֤�����json
	TOKEN_RESULT      stResult;
	nRet = ParserTokenResult(pRecvBuffer + SOCKET_HEAD_LEN, &stResult);
	if (nRet != 0)
	{
		return 0;
	}
	// �Ƿ���֤�ɹ�
	if (stResult.nResult != 0)
	{
		printf("token check error\n");
		return -1;
	}
	// ѡ������з����������Լ���׼������(Player -> Judger)
	READY_PARAM     stReady;
	strcpy(stReady.szToken, szToken);
	strcpy(stReady.szAction, "ready");
	memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = CreateReadyParam(&stReady, pSendBuffer, &nLen);
	if (nRet != 0)
	{
		return nRet;
	}
	nRet = SendJuderData(hSocket, pSendBuffer, nLen);
	if (nRet != 0)
	{
		return nRet;
	}
	//��ս��ʼ֪ͨ(Judger -> Player)��
	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}
	// ��������
	//Mapinfo �ṹ����ܴܺ󣬲�̫�ʺϷ���ջ�У����Զ���Ϊȫ�ֻ����ڴ����
	MAP_INFO            *pstMapInfo;
	MATCH_STATUS        *pstMatchStatus;
	FLAY_PLANE          *pstFlayPlane;
	pstMapInfo = (MAP_INFO *)malloc(sizeof(MAP_INFO));
	if (pstMapInfo == NULL)
	{
		return -1;
	}
	pstMatchStatus = (MATCH_STATUS *)malloc(sizeof(MATCH_STATUS));
	if (pstMapInfo == NULL)
	{
		return -1;
	}
	pstFlayPlane = (FLAY_PLANE *)malloc(sizeof(FLAY_PLANE));
	if (pstFlayPlane == NULL)
	{
		return -1;
	}
	memset(pstMapInfo, 0, sizeof(MAP_INFO));
	memset(pstMatchStatus, 0, sizeof(MATCH_STATUS));
	memset(pstFlayPlane, 0, sizeof(FLAY_PLANE));
	nRet = ParserMapInfo(pRecvBuffer + SOCKET_HEAD_LEN, pstMapInfo);
	if (nRet != 0)
	{
		return nRet;
	}
	// ��һ�ΰ����˻��ĳ�ʼ��ֵ��flayplane
	pstFlayPlane->nPurchaseNum = 0;
	pstFlayPlane->nUavNum = pstMapInfo->nUavNum;
	for (int i = 0; i < pstMapInfo->nUavNum; i++)
	{
		pstFlayPlane->astUav[i] = pstMapInfo->astUav[i];
	}
	printinformation(pstMapInfo);
	Gloablestatus gloablestatus;
	vector<Point3d> freepointsvector;
	Creatfreepointvector(pstMapInfo, freepointsvector);
	vector<Uav> uav;
	vector<Goods> goods;
	vector<int> rankeduavprice;
	vector<UAV_PRICE> uavprice;
	Rankuavprice(pstMapInfo, rankeduavprice,uavprice,gloablestatus);
	vector<vector<vector<char>>>  map3d(pstMapInfo->nMapZ, vector<vector<char>>(pstMapInfo->nMapX, vector<char>(pstMapInfo->nMapY, 0)));
	attractmap3d(pstMapInfo, map3d);//��ȡ3D��ͼ
	//printmap3d(pstMapInfo, map3d);
	vector<Point3d> path3d;
	vector<int> goodschoosed;

	gloablestatus.freepointsvector = freepointsvector;
	gloablestatus.numofdestroyeduav = 0;
	gloablestatus.maxfreepointsindex = 0;
	gloablestatus.isgoodsweightcounted = false;
	gloablestatus.indexofuavquenetoperchase = 0;
	gloablestatus.heightofworkinguav = pstMapInfo->nHLow;
	Initialization(uav, pstMapInfo, map3d, gloablestatus,uavprice);
	while (1)
	{
		// ���е�ǰʱ�̵����ݼ���, �����мƻ��ṹ�壬ע�⣺0ʱ�̲��ܽ����ƶ�������һ�ν����ѭ��ʱ
		if (pstMatchStatus->nTime != 0)
		{
			printf("************************************************************************************\n");
			/*if (pstMatchStatus->nTime == 1) { uav[0].isfree = true; uav[0].issetpath = true; }
			if (pstMatchStatus->nTime == 2) { uav[1].isfree = true; uav[1].issetpath = true; }
			if (pstMatchStatus->nTime == 3) { uav[2].isfree = true; uav[2].issetpath = true; }
			if (pstMatchStatus->nTime == 4) { uav[3].isfree = true; uav[3].issetpath = true; }
			if (pstMatchStatus->nTime == 5) { uav[4].isfree = true; uav[4].issetpath = true; }
			if (pstMatchStatus->nTime == 6) { uav[5].isfree = true; uav[5].issetpath = true; }*/
			if (pstMatchStatus->nTime == 20) {
			printf("stop");
			}
			if (pstMatchStatus->nTime == 180) {
			printf("stop");
			}
			if (pstMatchStatus->nTime == 300) {
			printf("stop");
			}
			if (pstMatchStatus->nTime == 400) {
			printf("stop");
			}
			//��ȡ��ͼ��Ϣ
			printf("��ȡ������Ϣ��\n");
			Getgoodsinfo(pstMatchStatus, gloablestatus, map3d, pstMapInfo);
			Getenemyinfo(pstMatchStatus, gloablestatus);
			Oneuavflyeachtimestamp(uav,gloablestatus,pstMatchStatus);
			//�����зɻ���������Լ�Ҫȥ���صĵл������ҹ滮·��
			printf("�����зɻ�����Ŀ�꣡\n");
			Assigntarget(uavprice,gloablestatus,pstMapInfo, pstMatchStatus, uav, map3d, goods, goodschoosed, rankeduavprice);
			//�ж��Ƿ�����ײ���������
			printf("������ײ��\n");
			Judgecrashandadjustpath(pstMapInfo, pstMatchStatus, map3d, uav);
			//�����úõ�·������Ϣ���͸�������
			printf("��ӷ��мƻ���\n");
			Addpathtoflyplan(goodschoosed, pstMapInfo, pstMatchStatus, pstFlayPlane, map3d, uav, gloablestatus);
			//�����µķɻ����ҳ�ʼ����Щ�ɻ�
			printf("����ɻ������г�ʼ����\n");
			Buyuavandinitial(uavprice,rankeduavprice, pstMapInfo, pstMatchStatus, uav, map3d, pstFlayPlane, gloablestatus);
		}
		//���ͷ��мƻ��ṹ��
		memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
		nRet = CreateFlayPlane(pstFlayPlane, szToken, pSendBuffer, &nLen);
		if (nRet != 0)
		{
			return nRet;
		}
		nRet = SendJuderData(hSocket, pSendBuffer, nLen);
		if (nRet != 0)
		{
			return nRet;
		}
		//printf("%s\n", pSendBuffer);
		// ���ܵ�ǰ����״̬
		memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
		nRet = RecvJuderData(hSocket, pRecvBuffer);
		if (nRet != 0)
		{
			return nRet;
		}
		// ����
		nRet = ParserMatchStatus(pRecvBuffer + SOCKET_HEAD_LEN, pstMatchStatus);
		if (nRet != 0)
		{
			return nRet;
		}
		//printf("%s\n", pRecvBuffer);

		if (pstMatchStatus->nMacthStatus == 1)
		{
			// ��������
			printf("game over, we value %d, enemy value %d\n", pstMatchStatus->nWeValue, pstMatchStatus->nEnemyValue);
			return 0;
		}
	}
	// �ر�socket
	OSCloseSocket(hSocket);
	// ��Դ����
	free(pRecvBuffer);
	free(pSendBuffer);
	free(pstMapInfo);
	free(pstMatchStatus);
	free(pstFlayPlane);
	return 0;
}
void Getgoodsinfo(MATCH_STATUS * pstMatch, Gloablestatus& gloablestatus, vector<vector<vector<char>>> map3d, MAP_INFO *pstMap) {
	for (int i = 0; i < pstMatch->nGoodsNum; i++) {
		map<int, Goods>::iterator it;
		it = gloablestatus.goodsmap.find(pstMatch->astGoods[i].nNO);
		if (it == gloablestatus.goodsmap.end()) {
			Goods goods;
			goods.num = pstMatch->astGoods[i].nNO;
			goods.value = pstMatch->astGoods[i].nValue;
			goods.weight = pstMatch->astGoods[i].nWeight;
			Point3d pstart(pstMatch->astGoods[i].nStartX,pstMatch->astGoods[i].nStartY,0);
			Point3d pend(pstMatch->astGoods[i].nEndX,pstMatch->astGoods[i].nEndY,0);
			vector<Point3d> pathstarttoend;
			pathstarttoend = Getgoodsstarttoend(pstart, pend, pstMatch, gloablestatus, map3d, pstMap);
			goods.pathstarttoend = pathstarttoend;
			goods.chargecost = goods.weight*(goods.pathstarttoend.size()+1);
			goods.pstart = pstart;
			goods.pend = pend;
			gloablestatus.goodsmap.insert(pair<int, Goods>(pstMatch->astGoods[i].nNO, goods));
		}
		else {

		}
	}
}
void Getenemyinfo(MATCH_STATUS * pstMatch, Gloablestatus& gloablestatus) {
	if (pstMatch->nTime == 1) {
		if (pstMatch->nUavEnemyNum == 0) {
			Point3d p(-1, -1, -1);
			gloablestatus.enemypoint = p;
		}
		else {
			Point3d p(pstMatch->astEnemyUav[0].nX, pstMatch->astEnemyUav[0].nY, pstMatch->astEnemyUav[0].nZ);
			gloablestatus.enemypoint = p;
		}
	}
	return;
}
void Oneuavflyeachtimestamp(vector<Uav>& uav,Gloablestatus& gloablestatus, MATCH_STATUS * pstMatch) {

	for (int i = 0; i < gloablestatus.timetostopuavs1.size(); i++) {
		if (pstMatch->nTime >= gloablestatus.timetostopuavs1[i] && pstMatch->nTime < gloablestatus.timetostopuavs2[i]) {

			return ;
		}
	}
	vector<Uav> uavtemp;
	for (int i = 0; i < uav.size(); i++) {
		if (uav[i].ischarging) {
			uavtemp.push_back(uav[i]);
		}
	}
	if (pstMatch->nTime > 10) {
		sort(uavtemp.begin(), uavtemp.end(), [](const Uav& uav1, const Uav& uav2) {return uav1.loadweight1 < uav2.loadweight1; });
	}
	else
	{
		sort(uavtemp.begin(), uavtemp.end(), [](const Uav& uav1, const Uav& uav2) {return uav1.loadweight1 > uav2.loadweight1; });
	}
	for (int i = 0; i < uavtemp.size(); i++) {
		if (uavtemp[i].isfullcharge) {
			uav[uavtemp[i].num].isfree = true;
			uav[uavtemp[i].num].ischarging = false;
			uav[uavtemp[i].num].issetpath = true;
			break;
		}
	}
}
//�ڵ�ͼ����һЩ����Ϊ�ɻ�����ͣ��ʱ��λ�ã�����ͼ����ƽ���ֳ�16�ݣ�ÿ��ÿ������4�����ɻ���ɢ����16�������е�����ص�
void Buyuavandinitial(vector<UAV_PRICE>& uavprice, vector<int> rankeduavprice, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, vector<vector<vector<char>>> map3d, FLAY_PLANE *pstFlayPlane, Gloablestatus& gloablestatus) {
	if (uav.size() >= gloablestatus.freepointsvector.size() || uav.size() - gloablestatus.numofdestroyeduav > MINNUMOFUAVWORKING || gloablestatus.maxfreepointsindex >= 16 * (pstMap->nHHigh - pstMap->nHLow + 1)) {
		pstFlayPlane->nPurchaseNum = 0;
		memset(pstFlayPlane->szPurchaseType, 0, sizeof(pstFlayPlane->szPurchaseType));
		return;
	}
	pstFlayPlane->nPurchaseNum = 0;
	memset(pstFlayPlane->szPurchaseType, 0, sizeof(pstFlayPlane->szPurchaseType));
	int index = Judgewhichuavtypetobuy(uavprice, rankeduavprice, pstMap, pstMatch, uav, gloablestatus);
	if (pstMatch->nWeValue >= rankeduavprice[rankeduavprice.size() - 1 - index]) {
		pstFlayPlane->nPurchaseNum = 1;
		strcpy(pstFlayPlane->szPurchaseType[0], uavprice[uavprice.size() - 1 - index].szType);
		printf("����ɻ�%d,�ͺţ�%s\n", uav.size(), uavprice[uavprice.size() - 1 - index].szType);
		if (uav.size() - gloablestatus.numofdestroyeduav > MINNUMOFUAVTOFIGHT) {
			gloablestatus.indexofuavquenetoperchase++;
		}
		Uav uavtemp(uav.size());
		uavtemp.loadweight1 = uavprice[uavprice.size() - 1 - index].nLoadWeight;
		uavtemp.capacity = uavprice[uavprice.size() - 1 - index].capacity;
		uavtemp.charge = uavprice[uavprice.size() - 1 - index].charge;
		uavtemp.value = uavprice[uavprice.size() - 1 - index].nValue;
		if (index > 0) {
			uavtemp.loadweight2 = uavprice[uavprice.size() - index].nLoadWeight;
		}
		else {
			uavtemp.loadweight2 = 0;
		}
		uav.push_back(uavtemp);
		Setnewuavinfo(pstMap, pstMatch, uav, map3d, 1, gloablestatus);
	}
	else {
		gloablestatus.ismoneyenough = false;
	}
/*for (int i = 0; i < pstMap->nUavPriceNum; i++) {
	if (pstMatch->nWeValue >= rankeduavprice[i]) {
		pstFlayPlane->nPurchaseNum = 1;
		for (int j = 0; j < pstMap->nUavPriceNum; j++) {
			if (rankeduavprice[i] == pstMap->astUavPrice[j].nValue) {

				strcpy(pstFlayPlane->szPurchaseType[0], pstMap->astUavPrice[j].szType);
			}
		}
		Uav uavtemp(uav.size());
		uav.push_back(uavtemp);
		Setnewuavinfo(pstMap, pstMatch, uav, map3d, 1, gloablestatus);
		return;
	}
}*/

}

void Assigntarget(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, vector<vector<vector<char>>> map3d, vector<Goods>& goods, vector<int> &goodschoosed, vector<int> rankeduavprice) {
	printf("ϵͳʱ��:%d\n", pstMatch->nTime);
	printf("pastmatch�У����������%d\n", pstMatch->nGoodsNum);

	for (int i = 0; i < pstMatch->nGoodsNum; i++) {
		//pstMatch->astGoods[i].ischoosed = false;
		//printf("pstmatch�У�����ı��%d\n", pstMatch->astGoods[i].nNO);
		//printf("pstmatch�У�����ʣ��ʱ��%d\n", pstMatch->astGoods[i].nLeftTime);
		//printf("����Ŀ�ʼ����λ�ã�%d,%d,%d->%d,%d,%d\n", pstMatch->astGoods[i].nStartX, pstMatch->astGoods[i].nStartY, 0, pstMatch->astGoods[i].nEndX, pstMatch->astGoods[i].nEndY, 0);
	}

	for (int i = 0; i < uav.size(); i++) {
		if (uav[i].isfighting&&pstMatch->astWeUav[uav[i].num].nStatus != 1) {
			bool booltemp = false;
			for (int j = 0; j < pstMatch->nUavEnemyNum; j++) {
				if (uav[i].idofenemyuavtofight == pstMatch->astEnemyUav[j].nNO) {
					booltemp = true;
					break;
				}
			}if (booltemp) {

			}
			else {
				printf("�ҷ��ɻ�%d��Ŀ��л�%d�Ѿ�ײ��!\n", uav[i].num, uav[i].idofenemyuavtofight);
				uav[i].isfree = true;
				uav[i].issetpath = true;
				uav[i].isfighting = false;
				uav[i].isdefended = false;
				uav[i].isdefended1 = false;
				uav[i].isdefending = false;
				uav[i].isdefending1 = false;
				uav[i].istotransporting = false;
				uav[i].istranspointing = false;
			}
		}
	}

	for (int i = 0; i < uav.size(); i++) {
		if (uav[i].isdefending&&pstMatch->astWeUav[uav[i].num].nStatus != 1) {
			if (pstMatch->astWeUav[uav[i].idofuavtodefend].nStatus == 1) {
				uav[i].isfree = true;
				uav[i].issetpath = true;
				uav[i].isdefending=false;
			}
		}
	}
	for (int i = 0; i < uav.size(); i++) {
		if (uav[i].isdefending1&&pstMatch->astWeUav[i].nStatus != 1) {
			if (pstMatch->astWeUav[uav[i].idofuavtodefend1].nStatus == 1||
				(uav[uav[i].idofuavtodefend1].istranspointing&&pstMatch->astWeUav[uav[i].idofuavtodefend1].nZ>=pstMap->nHLow)) {
				uav[i].isfree = true;
				uav[i].issetpath = true;
				uav[i].isdefending1 = false;
				uav[uav[i].idofuavtodefend1].isdefended1 = false;
			}
		}
	}

	//���ŷɻ�ս��
	vector<Uav> freeuav;
	for (size_t i = 0; i < uav.size(); i++) {
		if (uav[i].isfree && (pstMatch->astWeUav[uav[i].num].nStatus != 1) && pstMatch->astWeUav[uav[i].num].nZ >= pstMap->nHLow) {
			freeuav.push_back(uav[i]);
		}
	}
	for (size_t i = 0; i < uav.size(); i++)
	{
		if (!uav[i].isfree && !uav[i].istranspointing&&pstMatch->astWeUav[uav[i].num].nStatus != 1 && pstMatch->astWeUav[uav[i].num].nZ >= pstMap->nHLow
			&&!uav[i].isfighting){
			freeuav.push_back(uav[i]);
		}
	}
	printf("���ŷɻ�ս����\n");
	Assignuavtofight(uavprice, gloablestatus, freeuav, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//�����Լ��ķɻ�
	vector<Uav> freeuav3;
	for (int i = 0; i < uav.size(); i++) {
		if (pstMatch->astWeUav[uav[i].num].nStatus != 1 && pstMatch->astWeUav[uav[i].num].nZ >= pstMap->nHLow
			&&uav[i].value == uavprice[uavprice.size() - 1].nValue) {
			if (uav[i].isfree) {
				freeuav3.push_back(uav[i]);
			}
			else if (!uav[i].istranspointing && !uav[i].isdefending && !uav[i].isfighting&&!uav[i].isdefending1) {
				freeuav3.push_back(uav[i]);
			}
		}
	}
	printf("�����Լ��ķɻ���\n");
	Assignuavtodefence(uavprice, gloablestatus, freeuav3, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//���ɻ��������
	vector<Uav> freeuav1;
	for (size_t i = 0; i < uav.size(); i++) {
		if (uav[i].isfree && (pstMatch->astWeUav[uav[i].num].nStatus != 1)) {
			freeuav1.push_back(uav[i]);
		}
	}
	sort(freeuav1.begin(), freeuav1.end(), [](const Uav& uav1, const Uav& uav2) {return uav1.loadweight1 < uav2.loadweight1; });
	if (pstMatch->nTime > 5) {
		printf("���ɻ�������\n");
		Assignuavtogoods(uavprice, gloablestatus, freeuav1, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);
	}

	//���ɻ���������
	vector<Uav> freeuav2;
	if (pstMatch->nTime >= 6) {
		for (size_t i = 0; i < uav.size(); i++) {
			if (uav[i].istotransporting && (pstMatch->astWeUav[uav[i].num].nStatus != 1)&&pstMatch->nTime-uav[i].time_temp>0) {
				freeuav2.push_back(uav[i]);
			}
		}
	}
	printf("���ɻ��������\n");
	Adjustuavtogoods(uavprice, gloablestatus, freeuav2, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//����Ҫȥ���ͻ���ķɻ�
	vector<Uav> freeuav5;
	for (int i = 0; i < uav.size(); i++) {
		if (pstMatch->astWeUav[uav[i].num].nStatus != 1 && pstMatch->astWeUav[uav[i].num].nZ >= pstMap->nHLow
			&&uav[i].value == uavprice[uavprice.size() - 1].nValue) {
			if (uav[i].isfree) {
				freeuav5.push_back(uav[i]);
			}
			else if(!uav[i].istranspointing&&!uav[i].isfighting&&!uav[i].isdefending1){
				freeuav5.push_back(uav[i]);
			}
		}
	}
	printf("����Ҫȥ���ͻ���ķɻ���\n");
	Assignuavtoprotectuavtotransport(uavprice, gloablestatus, freeuav5, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//���ɻ����
	vector<Uav> freeuav4;
	for (int i = 0; i < uav.size(); i++) {
		if (uav[i].isfree&&pstMatch->astWeUav[uav[i].num].nStatus != 1 && !uav[i].isfullcharge&&uav[i].value > uavprice[uavprice.size() - 1].nValue
			&&pstMatch->astWeUav[uav[i].num].nZ>=pstMap->nHLow&&!uav[i].isbackinghome) {
			freeuav4.push_back(uav[i]);
		}
	}
	printf("���ɻ���磡\n");
	Assignuavtocharge(uavprice, gloablestatus, freeuav4, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);
	if (pstMatch->nTime == 123) {
		printf("stop\n");
	}
	if (pstMatch->nTime == 228) {
		printf("stop\n");
	}
	if (pstMatch->nTime == 317) {
		printf("stop\n");
	}
	printf("�����зɻ�����·����\n");
	for (size_t i = 0; i < uav.size(); i++) {
		if (uav[i].issetpath) {
			Setpath(gloablestatus,map3d, uav[i], pstMatch, pstMap);
			uav[i].issetpath = false;
		}
	}
	return;
}
void Addpathtoflyplan(vector<int> &goodschoosed, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<vector<vector<char>>> map3d, vector<Uav>& uav, Gloablestatus& gloablestatus) {
	//��ÿһ�ܷɻ�����ӷ��мƻ�
	/*for (int i = 0; i < uav.size(); i++) {
		printf("�ɻ�%d�ļ�ֵ��%s\n", pstMatch->astWeUav[uav[i].num].nNO, pstMatch->astWeUav[uav[i].num].szType);
	}*/
	if (Isnewuavdestoryed(pstMatch, uav, gloablestatus)) {
		memset(pstFlayPlane, 0, sizeof(FLAY_PLANE));
		Setplan(goodschoosed, pstMatch, pstFlayPlane, uav,gloablestatus);
	}
	else if (!Isnewuavdestoryed(pstMatch, uav, gloablestatus)) {
		Setplan(goodschoosed, pstMatch, pstFlayPlane, uav,gloablestatus);
	}
}
void Judgecrashandadjustpath(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav) {
	printf("��ֹ��ײ��������������������������������������������������������������������������\n");
	vector<Uav> candidateuav;
	for (int i = 0; i < uav.size(); i++) {
		if (pstMatch->astWeUav[uav[i].num].nStatus != 1 && pstMatch->astWeUav[uav[i].num].nZ >= pstMap->nHLow - 1) {
			candidateuav.push_back(uav[i]);
		}
	}

	if (candidateuav.size() >= 2) {
		for (int i = 0; i < candidateuav.size() - 1; i++) {
			for (int j = i + 1; j < candidateuav.size(); j++) {
				Point3d Anow(pstMatch->astWeUav[candidateuav[i].num].nX, pstMatch->astWeUav[candidateuav[i].num].nY, pstMatch->astWeUav[candidateuav[i].num].nZ);
				Point3d Anext(uav[candidateuav[i].num].path3d[pstMatch->nTime - uav[candidateuav[i].num].time_temp].x, uav[candidateuav[i].num].path3d[pstMatch->nTime - uav[candidateuav[i].num].time_temp].y, uav[candidateuav[i].num].path3d[pstMatch->nTime - uav[candidateuav[i].num].time_temp].z);
				Point3d Bnow(pstMatch->astWeUav[candidateuav[j].num].nX, pstMatch->astWeUav[candidateuav[j].num].nY, pstMatch->astWeUav[candidateuav[j].num].nZ);
				Point3d Bnext(uav[candidateuav[j].num].path3d[pstMatch->nTime - uav[candidateuav[j].num].time_temp].x, uav[candidateuav[j].num].path3d[pstMatch->nTime - uav[candidateuav[j].num].time_temp].y, uav[candidateuav[j].num].path3d[pstMatch->nTime - uav[candidateuav[j].num].time_temp].z);
				if (Istwouavcollide(Anow, Anext, Bnow, Bnext)) {
					Adjustpathfortwouav(uav[candidateuav[i].num], uav[candidateuav[j].num], pstMatch, pstMap, map3d);
				}
			}
		}
	}
	vector<Uav> candidateuav1;
	for (int i = 0; i < uav.size(); i++) {
		if (pstMatch->astWeUav[uav[i].num].nStatus != 1 && pstMatch->astWeUav[uav[i].num].nZ >= pstMap->nHLow - 1) {
			candidateuav1.push_back(uav[i]);
		}
	}
	if (candidateuav1.size() >= 2) {
		for (int i = 0; i < candidateuav1.size() - 1; i++) {
			for (int j = i + 1; j < candidateuav1.size(); j++) {
				Point3d Anow(pstMatch->astWeUav[candidateuav1[i].num].nX, pstMatch->astWeUav[candidateuav1[i].num].nY, pstMatch->astWeUav[candidateuav1[i].num].nZ);
				Point3d Anext(uav[candidateuav1[i].num].path3d[pstMatch->nTime - uav[candidateuav1[i].num].time_temp].x, uav[candidateuav1[i].num].path3d[pstMatch->nTime - uav[candidateuav1[i].num].time_temp].y, uav[candidateuav1[i].num].path3d[pstMatch->nTime - uav[candidateuav1[i].num].time_temp].z);
				Point3d Bnow(pstMatch->astWeUav[candidateuav1[j].num].nX, pstMatch->astWeUav[candidateuav1[j].num].nY, pstMatch->astWeUav[candidateuav1[j].num].nZ);
				Point3d Bnext(uav[candidateuav1[j].num].path3d[pstMatch->nTime - uav[candidateuav1[j].num].time_temp].x, uav[candidateuav1[j].num].path3d[pstMatch->nTime - uav[candidateuav1[j].num].time_temp].y, uav[candidateuav1[j].num].path3d[pstMatch->nTime - uav[candidateuav1[j].num].time_temp].z);
				if (Istwouavcollide(Anow, Anext, Bnow, Bnext)) {
					Adjustpathfortwouav(uav[candidateuav1[i].num], uav[candidateuav1[j].num], pstMatch, pstMap, map3d);
				}
			}
		}
	}
	return;
}

inline void Assignuavtofight(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice) {
	//ȥײ����
	vector<int> temp1, temp2;
	for (int i = 0; i < pstMatch->nUavEnemyNum; i++) {
		temp2.push_back(-1);
	}
	for (int i = 0; i < freeuav.size(); i++) {
		temp1.push_back(-1);
		//if (strcmp(pstMatch->astWeUav[freeuav[i].num].szType, uavprice[uavprice.size() - 1].szType) ) {
			for (int j = 0; j < pstMatch->nUavEnemyNum; j++) {
				if (pstMatch->astEnemyUav[j].nStatus != 2 && pstMatch->astEnemyUav[j].nGoodsNo != -1&&temp2[j]!=1&&!Isenemyuavchoosed(pstMatch->astEnemyUav[j].nNO,gloablestatus)
					&& gloablestatus.uavprice[pstMatch->astWeUav[freeuav[i].num].szType].nValue<=gloablestatus.uavprice[pstMatch->astEnemyUav[j].szType].nValue) {
					printf("%d<=%d\n", gloablestatus.uavprice[pstMatch->astWeUav[freeuav[i].num].szType].nValue, gloablestatus.uavprice[pstMatch->astEnemyUav[j].szType].nValue);
					Point3d pgoods(gloablestatus.goodsmap[pstMatch->astEnemyUav[j].nGoodsNo].pend.x, gloablestatus.goodsmap[pstMatch->astEnemyUav[j].nGoodsNo].pend.y, pstMap->nHLow);
					/*for (int k = 0; k < pstMatch->nGoodsNum; k++) {
						if (pstMatch->astGoods[k].nNO == pstMatch->astEnemyUav[j].nGoodsNo) {
							Point3d p3d(pstMatch->astGoods[k].nEndX, pstMatch->astGoods[k].nEndY, pstMap->nHLow);
							pgoods = p3d;
							break;
						}
					}*/
					Point3d pwuav(pstMatch->astWeUav[freeuav[i].num].nX, pstMatch->astWeUav[freeuav[i].num].nY, pstMatch->astWeUav[freeuav[i].num].nZ);
					Point3d peuav(pstMatch->astEnemyUav[j].nX, pstMatch->astEnemyUav[j].nY, pstMatch->astEnemyUav[j].nZ);
					if (Caculateclosetflydistanceoftwopoints(pwuav, pgoods) <= Caculateclosetflydistanceoftwopoints(peuav, pgoods)) {
						temp1[i] = j;
						temp2[j] = 1;
						gloablestatus.enemyuavchoosed.push_back(pstMatch->astEnemyUav[j].nNO);
						uav[freeuav[i].num].fightingpoint = pgoods;
						uav[freeuav[i].num].isbackinghome = false;
						uav[freeuav[i].num].isfighting = true;
						uav[freeuav[i].num].isfree = false;

						uav[freeuav[i].num].issetpath = true;
						uav[freeuav[i].num].isdefending = false;
						uav[freeuav[i].num].isdefending1 = false;
						uav[freeuav[i].num].istotransporting = false;
						uav[freeuav[i].num].idofenemyuavtofight = pstMatch->astEnemyUav[j].nNO;
						printf("ս����%d����ս�����ɻ��ĵ�ǰλ��:%d,%d,%d��Ŀ��л�:%d���л�λ��:%d,%d,%d��Ŀ�����λ��:%d,%d,%d\n", freeuav[i].num,pwuav.x,pwuav.y,pwuav.z, pstMatch->astEnemyUav[j].nNO,peuav.x,peuav.y,peuav.z, pgoods.x, pgoods.y, pgoods.z);
						if (uav[freeuav[i].num].goodstotransport != -1) {
							uav[freeuav[i].num].goodstotransport = -1;
							vector<int>::iterator it = goodschoosed.begin();
							for (; it != goodschoosed.end();)
							{
								if (*it == uav[freeuav[i].num].goodstotransport)
									//ɾ��ָ��Ԫ�أ�����ָ��ɾ��Ԫ�ص���һ��Ԫ�ص�λ�õĵ�����
									it = goodschoosed.erase(it);
								else
									//������ָ����һ��Ԫ��λ��
									++it;
							}
						}
						break;
					}
				}
			}
		//}
	}
}

//�������г��ֵĻ�������Ƿ���ɻ�ȥȡ����
inline void Assignuavtogoods(vector<UAV_PRICE>& uavprice,Gloablestatus& gloablestatus,vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice) {

	printf("�����зɻ��������!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	vector<int> temp1, temp2,temp3,temp5,temp6;//��ʱ�������ֱ������洢�����Ӧ�ɻ��Ĳ����������Ӧ�ɻ��ı�ţ��Լ��ɻ��Ƿ�ѡ���˵ı�־
	vector<vector<Point3d>> temp4;
	vector<vector<int>> costmatrix;

	//��ÿһ�ܿ��еķɻ�Ѱ�һ���
	//if (pstMatch->nTime >= pstMap->nHLow + 1 + STARTFINDINGGOODS_TIME || pstMatch->nGoodsNum >= 20) {
		for (int i = 0; i < freeuav.size(); i++) {
			temp1.push_back(100000);
			temp2.push_back(-1);
			temp3.push_back(-1);
			temp5.push_back(-1);
			temp6.push_back(-1);
			temp4.push_back({});
			costmatrix.push_back({});
			//if (pstMatch->nTime - i >= pstMap->nHLow + 1 + STARTFINDINGGOODS_TIME) {
				//printf("���ɻ�%dѰ�һ���\n", pstMatch->astWeUav[freeuav[i].num].nNO);
				for (int j = 0; j < pstMatch->nGoodsNum; j++) {
					if (!Isgoodschoosed(pstMatch->astGoods[j].nNO, goodschoosed)) {
						if (pstMatch->astGoods[j].nWeight <=freeuav[i].loadweight1 ) {
							Point3d startpoint(pstMatch->astWeUav[freeuav[i].num].nX, pstMatch->astWeUav[freeuav[i].num].nY, pstMatch->astWeUav[freeuav[i].num].nZ);//�ɻ���ǰ����
							Point3d endpoint(pstMatch->astGoods[j].nStartX, pstMatch->astGoods[j].nStartY, 0);//�����������
							Point3d goodsendpoint(pstMatch->astGoods[j].nEndX, pstMatch->astGoods[j].nEndX, 0);//�����յ�����
							int fasteststep;//�ɻ���������첽�����������ϰ�������
							bool ifhigher = uav[freeuav[i].num].value != uavprice[uavprice.size() - 1].nValue;
							int closestdistance = Caculate3ddistance(gloablestatus,startpoint, endpoint, goodsendpoint, fasteststep,ifhigher);
							int invgoodsrelevatevalue = 500*closestdistance/pstMatch->astGoods[j].nValue;
							if (fasteststep <= pstMatch->astGoods[j].nLeftTime&&gloablestatus.goodsmap[pstMatch->astGoods[j].nNO].chargecost <=pstMatch->astWeUav[freeuav[i].num].remain_electricity) {
								costmatrix[i].push_back(pstMatch->astGoods[j].nValue * 100 / closestdistance);
							//if (closestdistance < temp1[i] && fasteststep <= pstMatch->astGoods[j].nLeftTime&&fasteststep < UAVTOGOODSSTEP_UP && iftransportgoods) {
								//temp1[i] = closestdistance;
								if (invgoodsrelevatevalue < temp1[i]) {
									temp1[i] = invgoodsrelevatevalue;
									int workingheight = gloablestatus.heightofworkinguav + (ifhigher? 1 : 0);
									vector<Point3d> p3d = Caculateclosestpath(startpoint, endpoint, map3d, pstMap,workingheight);
									int p3dsize = p3d.size();
									if (p3dsize <= pstMatch->astGoods[j].nLeftTime) {
										temp2[i] = j;//��ʾ��i�ܷɻ���Ӧ��j������
										temp3[i] = pstMatch->astGoods[j].nValue;
										temp5[i] = pstMatch->astGoods[j].nWeight;
										temp4[i] = p3d;
										temp6[i] = temp3[i] * 100 / closestdistance;
									}
								}
							}
							else
							{
								costmatrix[i].push_back(0);
							}
						}
						else {
							costmatrix[i].push_back(0);
						}
					}
					else {
						costmatrix[i].push_back(0);
					}
				}
				if (temp2[i] != -1) {
					goodschoosed.push_back(pstMatch->astGoods[temp2[i]].nNO);
				}
			//}
		}
		/*printf("costmatrix:\n");
		for (int i = 0; i < freeuav.size(); i++) {
			for (int j = 0; j < pstMatch->nGoodsNum; j++) {
				printf("%d ", costmatrix[i][j]);
			}
			printf("\n");
		}*/
		//����ÿһ���ɻ���������״̬�Լ�Ҫȥȡ�Ļ���
		for (int i = 0; i < freeuav.size(); i++) {
			if (temp2[i] != -1) {
				printf("�ɻ�%d��Ӧ�Ļ��%d\n", freeuav[i].num, pstMatch->astGoods[temp2[i]].nNO);
				uav[freeuav[i].num].issetpath = true;
				uav[freeuav[i].num].isfree = false;
				uav[freeuav[i].num].isstoped = false;

				uav[freeuav[i].num].pathtogoods = temp4[i];
				uav[freeuav[i].num].goodsvalue = temp3[i];
				uav[freeuav[i].num].goodsweight = temp5[i];
				uav[freeuav[i].num].istotransporting = true;
				uav[freeuav[i].num].isdefended = false;
				uav[freeuav[i].num].isdefended1 = false;
				uav[freeuav[i].num].isbackinghome = false;
				uav[freeuav[i].num].isdefending = false;
				uav[freeuav[i].num].isdefending1 = false;
				uav[freeuav[i].num].goodsrelevatevalue = temp6[i];
				uav[freeuav[i].num].goodsposition.x = pstMatch->astGoods[temp2[i]].nStartX;
				uav[freeuav[i].num].goodsposition.y = pstMatch->astGoods[temp2[i]].nStartY;
				uav[freeuav[i].num].goodsposition.z = 0;
				uav[freeuav[i].num].targetposition.x = pstMatch->astGoods[temp2[i]].nEndX;
				uav[freeuav[i].num].targetposition.y = pstMatch->astGoods[temp2[i]].nEndY;
				uav[freeuav[i].num].targetposition.z = 0;
				uav[freeuav[i].num].goodstotransport = pstMatch->astGoods[temp2[i]].nNO;
				uav[freeuav[i].num].time_temp = pstMatch->nTime;
			}
		}
	//}
}
inline void Adjustuavtogoods(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice) {
	for (int i = 0; i < freeuav.size(); i++) {
		for (int j = 0; j < pstMatch->nGoodsNum; j++) {
			if (!Isgoodschoosed(pstMatch->astGoods[j].nNO, goodschoosed)&& pstMatch->astGoods[j].nWeight <= freeuav[i].loadweight1) {
				Point3d startpoint(pstMatch->astWeUav[freeuav[i].num].nX, pstMatch->astWeUav[freeuav[i].num].nY, pstMatch->astWeUav[freeuav[i].num].nZ);//�ɻ���ǰ����
				Point3d endpoint(pstMatch->astGoods[j].nStartX, pstMatch->astGoods[j].nStartY, 0);//�����������
				Point3d goodsendpoint(pstMatch->astGoods[j].nEndX, pstMatch->astGoods[j].nEndY, 0);//�����յ�����
				int fasteststep;
				bool ifhigher = uav[freeuav[i].num].value != uavprice[uavprice.size() - 1].nValue;
				int closestdistance = Caculate3ddistance(gloablestatus, startpoint, endpoint, goodsendpoint, fasteststep,ifhigher);
				int invgoodsrelevatevalue1 = 500 * closestdistance / pstMatch->astGoods[j].nValue;
				int invgoodsrelevatevalue2 = 500 * (uav[freeuav[i].num].path3d.size() - (pstMatch->nTime - uav[freeuav[i].num].time_temp)) / uav[freeuav[i].num].goodsvalue;
				//if (fasteststep <= pstMatch->astGoods[j].nLeftTime&&iftransportgoods&&(closestdistance<uav[freeuav[i].num].path3d.size()-(pstMatch->nTime - uav[freeuav[i].num].time_temp))) {
				if (fasteststep <= pstMatch->astGoods[j].nLeftTime && invgoodsrelevatevalue1<invgoodsrelevatevalue2&& gloablestatus.goodsmap[pstMatch->astGoods[j].nNO].chargecost<= pstMatch->astWeUav[freeuav[i].num].remain_electricity) {
					if (uav[freeuav[i].num].isdefended1) {
						uav[uav[freeuav[i].num].idofdefendinguav1].isdefending1 = false;
						uav[uav[freeuav[i].num].idofdefendinguav1].idofuavtodefend1 = -1;
						uav[freeuav[i].num].isdefended1 = false;
					}
					uav[freeuav[i].num].isdefended = false;
					uav[freeuav[i].num].isdefending = false;
					uav[freeuav[i].num].isdefending1 = false;
					uav[freeuav[i].num].issetpath = true;
					uav[freeuav[i].num].istotransporting = true;
					uav[freeuav[i].num].isbackinghome = false;
					int workingheight = gloablestatus.heightofworkinguav + (ifhigher ? 1:0);
					vector<Point3d> p3d = Caculateclosestpath(startpoint, endpoint, map3d, pstMap,workingheight);
					uav[freeuav[i].num].pathtogoods = p3d;
					uav[freeuav[i].num].goodsposition = endpoint;
					uav[freeuav[i].num].targetposition = goodsendpoint;
					uav[freeuav[i].num].time_temp = pstMatch->nTime;
					uav[freeuav[i].num].goodsweight = pstMatch->astGoods[j].nWeight;
					uav[freeuav[i].num].goodsvalue = pstMatch->astGoods[j].nValue;
					uav[freeuav[i].num].goodsrelevatevalue = pstMatch->astGoods[j].nValue * 100 / closestdistance;
					vector<int>::iterator it = goodschoosed.begin();
					for (; it != goodschoosed.end();)
					{
						if (*it == uav[freeuav[i].num].goodstotransport)
							//ɾ��ָ��Ԫ�أ�����ָ��ɾ��Ԫ�ص���һ��Ԫ�ص�λ�õĵ�����
							it = goodschoosed.erase(it);
						else
							//������ָ����һ��Ԫ��λ��
							++it;
					}
					goodschoosed.push_back(pstMatch->astGoods[j].nNO);
					uav[freeuav[i].num].goodstotransport = pstMatch->astGoods[j].nNO;
					printf("���ɻ�%d���·������%d\n", uav[freeuav[i].num].num, pstMatch->astGoods[j].nNO);
					break;
				}
			}
		}
	}
}
inline void Assignuavtodefence(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice) {
	int temp = 10000;
	vector<int> temp1;
	vector<Point3d> ptemp;
	for (int i = 0; i < uav.size(); i++) {
		temp1.push_back(-1);
		Point3d p(-1, -1, -1);
		ptemp.push_back(p);
		if (uav[i].istranspointing&&pstMatch->astWeUav[uav[i].num].nStatus != 1 && !uav[i].isdefended) {
			for (int j = 0; j < freeuav.size(); j++) {
				if (!uav[freeuav[j].num].isdefending&&((uav[i].value+gloablestatus.goodsmap[uav[i].goodstotransport].value)>=5*freeuav[j].value)) {
					Point3d pgoods(gloablestatus.goodsmap[uav[i].goodstotransport].pend.x, gloablestatus.goodsmap[uav[i].goodstotransport].pend.y, pstMap->nHLow);
					Point3d pwuav(pstMatch->astWeUav[freeuav[j].num].nX, pstMatch->astWeUav[freeuav[j].num].nY, pstMatch->astWeUav[freeuav[j].num].nZ);
					Point3d pwuavworking(pstMatch->astWeUav[uav[i].num].nX, pstMatch->astWeUav[uav[i].num].nY, pstMatch->astWeUav[uav[i].num].nZ);
					int l1 = Caculateclosetflydistanceoftwopoints(pwuav, pgoods);
					int l2 = Caculateclosetflydistanceoftwopoints(pwuavworking, pgoods);
					if (l1 < l2) {
						if (l1 < temp) {
							temp = l1;
							temp1[i] = j;
							ptemp[i] = pgoods;
						}
					}
				}
			}
			if (temp1[i] != -1) {
				uav[freeuav[temp1[i]].num].istotransporting = false;
				uav[freeuav[temp1[i]].num].isdefending = true;
				uav[freeuav[temp1[i]].num].issetpath = true;
				uav[freeuav[temp1[i]].num].isbackinghome = false;
				uav[i].isdefended = true;
				uav[i].idofdefendinguav = freeuav[temp1[i]].num;
				uav[freeuav[temp1[i]].num].idofuavtodefend = uav[i].num;
				uav[freeuav[temp1[i]].num].isfree = false;
				uav[freeuav[temp1[i]].num].defendingpoint = ptemp[i];
				printf("�Ż�%d�����������������Ŀ���!\n",uav[freeuav[temp1[i]].num].num);
			}
		}
	}
}
inline void Assignuavtoprotectuavtotransport(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice) {
	int temp = 10000;
	vector<int> temp1;
	vector<Point3d> ptemp;
	for (int i = 0; i < uav.size(); i++) {
		temp1.push_back(-1);
		Point3d p(-1, -1, -1);
		ptemp.push_back(p);
		if (uav[i].istotransporting&&pstMatch->astWeUav[uav[i].num].nStatus != 1 && !uav[i].isdefended1 ) {
			for (int j = 0; j < freeuav.size(); j++) {
				if (!uav[freeuav[j].num].isdefending1 && ((uav[i].value + gloablestatus.goodsmap[uav[i].goodstotransport].value) >= 5 * freeuav[j].value)) {
					Point3d pgoods(gloablestatus.goodsmap[uav[i].goodstotransport].pstart.x, gloablestatus.goodsmap[uav[i].goodstotransport].pstart.y, pstMap->nHLow);
					Point3d pwuav(pstMatch->astWeUav[freeuav[j].num].nX, pstMatch->astWeUav[freeuav[j].num].nY, pstMatch->astWeUav[freeuav[j].num].nZ);
					Point3d pwuavworking(pstMatch->astWeUav[uav[i].num].nX, pstMatch->astWeUav[uav[i].num].nY, pstMatch->astWeUav[uav[i].num].nZ);
					int l1 = Caculateclosetflydistanceoftwopoints(pwuav, pgoods);
					int l2 = Caculateclosetflydistanceoftwopoints(pwuavworking, pgoods);
					if (l1 < l2) {
						if (l1 < temp) {
							temp = l1;
							temp1[i] = j;
							ptemp[i] = pgoods;
						}
					}
				}
			}
			if (temp1[i] != -1) {
				uav[freeuav[temp1[i]].num].istotransporting = false;
				if (uav[freeuav[temp1[i]].num].isdefending) {
					uav[freeuav[temp1[i]].num].isdefending = false;
					uav[uav[freeuav[temp1[i]].num].idofuavtodefend].isdefended = false;
					uav[freeuav[temp1[i]].num].idofuavtodefend = -1;
				}

				uav[freeuav[temp1[i]].num].isdefending1 = true;
				uav[freeuav[temp1[i]].num].issetpath = true;
				uav[freeuav[temp1[i]].num].isbackinghome = false;
				uav[i].isdefended1 = true;
				uav[i].idofdefendinguav1 = freeuav[temp1[i]].num;
				uav[freeuav[temp1[i]].num].idofuavtodefend1 = uav[i].num;
				uav[freeuav[temp1[i]].num].isfree = false;
				uav[freeuav[temp1[i]].num].defendingpoint1 = ptemp[i];
				printf("�Ż�%d����������������������!\n", uav[freeuav[temp1[i]].num].num);
			}
		}
	}
}
inline void Assignuavtocharge(vector<UAV_PRICE>& uavprice, Gloablestatus& gloablestatus, vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice) {
	for (int i = 0; i < freeuav.size(); i++) {
		uav[freeuav[i].num].isbackinghome = true;
		uav[freeuav[i].num].isfree = false;
		uav[freeuav[i].num].issetpath = true;
		uav[freeuav[i].num].time_temp = pstMatch->nTime;
		printf("�ɻ�%d�ؼ�\n",uav[freeuav[i].num].num);
		break;
	}
}
inline void Setpath(Gloablestatus& gloablestatus,vector<vector<vector<char>>> map3d, Uav &uav, MATCH_STATUS * pstMatch, MAP_INFO *pstMap) {
	if (uav.isbackinghome) {
			Astar astar;
			astar.InitAstar(map3d[pstMatch->astWeUav[uav.num].nZ]);
			Point start(pstMatch->astWeUav[uav.num].nX, pstMatch->astWeUav[uav.num].nY);
			Point end(pstMap->astUav[0].nX, pstMap->astUav[0].nY);
			list<Point *> path = astar.GetPath(start, end, true);
			vector<Point3d> path3d;
			if (path.size() > 0) {
				if (path.size() > pstMatch->astWeUav[uav.num].nZ) {
					for (auto &p : path) {
						Point3d p3d(p->x, p->y, pstMatch->astWeUav[uav.num].nZ);
						path3d.push_back(p3d);
					}
					for (int i = pstMatch->astWeUav[uav.num].nZ; i > 0; i--) {
						Point3d p3d(pstMap->astUav[0].nX, pstMap->astUav[0].nY, i - 1);
						path3d.push_back(p3d);
					}
					gloablestatus.timetostopuavs1.push_back(uav.time_temp + path.size() - pstMatch->astWeUav[uav.num].nZ) ;
					gloablestatus.timetostopuavs2.push_back(uav.time_temp + path.size() + pstMatch->astWeUav[uav.num].nZ+4);
				}
				else {
					path.pop_back();
					int l = path.size();
					for (auto &p : path) {
						Point3d p3d(p->x, p->y, pstMatch->astWeUav[uav.num].nZ);
						path3d.push_back(p3d);
					}
					Point3d temp = *(path3d.end() - 1);
					for (int i = 0; i < pstMatch->astWeUav[uav.num].nZ - l; i++) {
						path3d.push_back(temp);
					}
					for (int i = pstMatch->astWeUav[uav.num].nZ+1; i > 0; i--) {
						Point3d p3d(pstMap->astUav[0].nX, pstMap->astUav[0].nY, i - 1);
						path3d.push_back(p3d);
					}
					gloablestatus.timetostopuavs1.push_back(uav.time_temp);
					gloablestatus.timetostopuavs2.push_back(uav.time_temp+2* pstMatch->astWeUav[uav.num].nZ+4);
				}
			}
			else if (start.x == end.x&&start.y == end.y) {
				for (int i = pstMatch->astWeUav[uav.num].nZ; i > 0; i--) {
					Point3d p3d(end.x, end.y, i - 1);
					path3d.push_back(p3d);
				}
			}
			uav.path3d.clear();
			uav.path3d = path3d;
			printf("�ɻ�%d�ؼҵĹ켣��", pstMatch->astWeUav[uav.num].nNO);
			/*for (int j = 0; j < uav.path3d.size(); j++) {
				printf("%d,%d,%d", uav.path3d[j].x, uav.path3d[j].y, uav.path3d[j].z);
				if (j < uav.path3d.size() - 1) {
					printf("->");
				}
				else {
					printf("\n");
				}
			}*/
	}
	else if (uav.isfree) {
			Point start(pstMatch->astWeUav[uav.num].nX, pstMatch->astWeUav[uav.num].nY);
			Point end(uav.freetimeposition.x, uav.freetimeposition.y);
			vector<Point3d> path3d;
			if(start.x==end.x&&start.y==end.y){
				if (uav.freetimeposition.z == pstMatch->astWeUav[uav.num].nZ) {

				}
				else if (pstMatch->astWeUav[uav.num].nZ < uav.freetimeposition.z) {
					for (int i = pstMatch->astWeUav[uav.num].nZ; i < uav.freetimeposition.z - 1; i++) {
						Point3d p3d(uav.freetimeposition.x, uav.freetimeposition.y, i + 1);
						path3d.push_back(p3d);
					}
				}
				else if (pstMatch->astWeUav[uav.num].nZ > uav.freetimeposition.z) {
					for (int i = pstMatch->astWeUav[uav.num].nZ; i > uav.freetimeposition.z + 1; i--) {
						Point3d p3d(uav.freetimeposition.x, uav.freetimeposition.y, i - 1);
						path3d.push_back(p3d);
					}
				}
			}
			else {
				Astar astar;
				astar.InitAstar(map3d[uav.freetimeposition.z]);
				list<Point *> path = astar.GetPath(start, end, true);

				if (path.size() > 0) {
					if (pstMatch->astWeUav[uav.num].nZ == uav.freetimeposition.z) {

					}
					else if (pstMatch->astWeUav[uav.num].nZ > uav.freetimeposition.z) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i > uav.freetimeposition.z + 1; i--) {
							Point3d p3d(pstMatch->astWeUav[uav.num].nX, pstMatch->astWeUav[uav.num].nY, i - 1);
							path3d.push_back(p3d);
						}
					}
					else if (pstMatch->astWeUav[uav.num].nZ < uav.freetimeposition.z) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i < uav.freetimeposition.z - 1; i++) {
							Point3d p3d(pstMatch->astWeUav[uav.num].nX, pstMatch->astWeUav[uav.num].nY, i + 1);
							path3d.push_back(p3d);
						}
					}
					for (auto &p : path) {
						Point3d p3d(p->x, p->y, uav.freetimeposition.z);
						path3d.push_back(p3d);
					}
				}
			}

			for (int i = 0; i < 1500; i++) {
				Point3d p3d(uav.freetimeposition.x, uav.freetimeposition.y, uav.freetimeposition.z);
				path3d.push_back(p3d);
			}
			uav.path3d.clear();
			uav.path3d = path3d;
			uav.time_temp = pstMatch->nTime;

			printf("�ɻ�%d���ɷ��еĹ켣��", pstMatch->astWeUav[uav.num].nNO);
			/*for (int j = 0; j < uav.path3d.size(); j++) {
				printf("%d,%d,%d", uav.path3d[j].x, uav.path3d[j].y, uav.path3d[j].z);
				if (j < uav.path3d.size() - 1) {
					printf("->");
				}
				else {
					printf("\n");
				}
			}*/

	}
	else if (!uav.isfree) {
		if (uav.isfighting) {//��ʼ�滮ս��·��������ս���ص㲢�ұ��ֲ���

				Point start(pstMatch->astWeUav[uav.num].nX, pstMatch->astWeUav[uav.num].nY);
				Point end(uav.fightingpoint.x, uav.fightingpoint.y);
				Astar astar;
				astar.InitAstar(map3d[pstMap->nHLow]);
				list<Point *> path = astar.GetPath(start, end, true);
				vector<Point3d> path3d;
				if (path.size() > 0) {
					path.pop_front();
					for (auto &p : path) {
						Point3d p3d(p->x, p->y, pstMatch->astWeUav[uav.num].nZ);
						path3d.push_back(p3d);
					}
					if (pstMatch->astWeUav[uav.num].nZ == pstMap->nHLow) {

					}
					else if (pstMatch->astWeUav[uav.num].nZ > pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i > pstMap->nHLow ; i--) {
							Point3d p3d(end.x, end.y, i - 1);
							path3d.push_back(p3d);
						}
					}
					else if (pstMatch->astWeUav[uav.num].nZ < pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i < pstMap->nHLow ; i++) {
							Point3d p3d(end.x, end.y, i + 1);
							path3d.push_back(p3d);
						}
					}
				}
				else if (pstMatch->astWeUav[uav.num].nX == uav.fightingpoint.x&&pstMatch->astWeUav[uav.num].nY == uav.fightingpoint.y) {
					if (pstMap->nHLow == pstMatch->astWeUav[uav.num].nZ) {

					}
					else if (pstMatch->astWeUav[uav.num].nZ < pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i < pstMap->nHLow - 1; i++) {
							Point3d p3d(uav.fightingpoint.x, uav.fightingpoint.y, i + 1);
							path3d.push_back(p3d);
						}
					}
					else if (pstMatch->astWeUav[uav.num].nZ > pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i > pstMap->nHLow + 1; i--) {
							Point3d p3d(uav.fightingpoint.x, uav.fightingpoint.y, i - 1);
							path3d.push_back(p3d);
						}
					}
				}
				/*Point3d p3d1(uav.fightingpoint.x, uav.fightingpoint.y, pstMap->nHLow);
				path3d.push_back(p3d1);*/
				Point3d p3d2(uav.fightingpoint.x, uav.fightingpoint.y, pstMap->nHLow-1);
				path3d.push_back(p3d2);
				for (int i = 0; i < 1500; i++) {
					Point3d p3d(uav.fightingpoint.x, uav.fightingpoint.y, pstMap->nHLow-2);
					path3d.push_back(p3d);
				}
				uav.path3d.clear();
				uav.path3d = path3d;
				uav.time_temp = pstMatch->nTime;
				printf("�ɻ�%dս���Ĺ켣��", pstMatch->astWeUav[uav.num].nNO);
				/*for (int j = 0; j < uav.path3d.size(); j++) {
					printf("%d,%d,%d", uav.path3d[j].x, uav.path3d[j].y, uav.path3d[j].z);
					if (j < uav.path3d.size() - 1) {
						printf("->");
					}
					else {
						printf("\n");
					}
				}*/

		}
		else if (uav.isdefending) {

				Point start(pstMatch->astWeUav[uav.num].nX, pstMatch->astWeUav[uav.num].nY);
				Point end(uav.defendingpoint.x, uav.defendingpoint.y);
				vector<Point3d> path3d;
				if (start.x == end.x&&start.y == end.y) {
					if (pstMap->nHLow == pstMatch->astWeUav[uav.num].nZ) {

					}
					else if (pstMatch->astWeUav[uav.num].nZ < pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i < pstMap->nHLow - 1; i++) {
							Point3d p3d(uav.defendingpoint.x, uav.defendingpoint.y, i + 1);
							path3d.push_back(p3d);
						}
					}
					else if (pstMatch->astWeUav[uav.num].nZ > pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i > pstMap->nHLow + 1; i--) {
							Point3d p3d(uav.defendingpoint.x, uav.defendingpoint.y, i - 1);
							path3d.push_back(p3d);
						}
					}
				}
				else {
					Astar astar;
					astar.InitAstar(map3d[pstMap->nHLow]);
					list<Point *> path = astar.GetPath(start, end, true);

					if (path.size() > 0) {
						path.pop_front();
						for (auto &p : path) {
							Point3d p3d(p->x, p->y, pstMatch->astWeUav[uav.num].nZ);
							path3d.push_back(p3d);
						}
						if (pstMatch->astWeUav[uav.num].nZ == pstMap->nHLow) {

						}
						else if (pstMatch->astWeUav[uav.num].nZ > pstMap->nHLow) {
							for (int i = pstMatch->astWeUav[uav.num].nZ; i > pstMap->nHLow + 1; i--) {
								Point3d p3d(end.x, end.y, i - 1);
								path3d.push_back(p3d);
							}
						}
						else if (pstMatch->astWeUav[uav.num].nZ < pstMap->nHLow) {
							for (int i = pstMatch->astWeUav[uav.num].nZ; i < pstMap->nHLow - 1; i++) {
								Point3d p3d(end.x, end.y, i + 1);
								path3d.push_back(p3d);
							}
						}
					}
				}
				for (int i = 0; i < 1500; i++) {
					Point3d p3d(uav.defendingpoint.x, uav.defendingpoint.y, pstMap->nHLow);
					path3d.push_back(p3d);
				}
				uav.path3d.clear();
				uav.path3d = path3d;
				uav.time_temp = pstMatch->nTime;

				printf("�ɻ�%d�����Ĺ켣��", pstMatch->astWeUav[uav.num].nNO);
				/*for (int j = 0; j < uav.path3d.size(); j++) {
					printf("%d,%d,%d", uav.path3d[j].x, uav.path3d[j].y, uav.path3d[j].z);
					if (j < uav.path3d.size() - 1) {
						printf("->");
					}
					else {
						printf("\n");
					}
				}*/

		}
		else if (uav.isdefending1) {
			Point start(pstMatch->astWeUav[uav.num].nX, pstMatch->astWeUav[uav.num].nY);
			Point end(uav.defendingpoint1.x, uav.defendingpoint1.y);
			vector<Point3d> path3d;
			if (start.x == end.x&&start.y == end.y) {
				if (pstMap->nHLow == pstMatch->astWeUav[uav.num].nZ) {

				}
				else if (pstMatch->astWeUav[uav.num].nZ < pstMap->nHLow) {
					for (int i = pstMatch->astWeUav[uav.num].nZ; i < pstMap->nHLow - 1; i++) {
						Point3d p3d(uav.defendingpoint1.x, uav.defendingpoint1.y, i + 1);
						path3d.push_back(p3d);
					}
				}
				else if (pstMatch->astWeUav[uav.num].nZ > pstMap->nHLow) {
					for (int i = pstMatch->astWeUav[uav.num].nZ; i > pstMap->nHLow + 1; i--) {
						Point3d p3d(uav.defendingpoint1.x, uav.defendingpoint1.y, i - 1);
						path3d.push_back(p3d);
					}
				}
			}
			else {
				Astar astar;
				astar.InitAstar(map3d[pstMap->nHLow]);
				list<Point *> path = astar.GetPath(start, end, true);
				if (path.size() > 0) {
					path.pop_front();
					for (auto &p : path) {
						Point3d p3d(p->x, p->y, pstMatch->astWeUav[uav.num].nZ);
						path3d.push_back(p3d);
					}
					if (pstMatch->astWeUav[uav.num].nZ == pstMap->nHLow) {

					}
					else if (pstMatch->astWeUav[uav.num].nZ > pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i > pstMap->nHLow + 1; i--) {
							Point3d p3d(end.x, end.y, i - 1);
							path3d.push_back(p3d);
						}
					}
					else if (pstMatch->astWeUav[uav.num].nZ < pstMap->nHLow) {
						for (int i = pstMatch->astWeUav[uav.num].nZ; i < pstMap->nHLow - 1; i++) {
							Point3d p3d(end.x, end.y, i + 1);
							path3d.push_back(p3d);
						}
					}
				}
			}
			for (int i = 0; i < 1500; i++) {
				Point3d p3d(uav.defendingpoint1.x, uav.defendingpoint1.y, pstMap->nHLow);
				path3d.push_back(p3d);
			}
			uav.path3d.clear();
			uav.path3d = path3d;
			uav.time_temp = pstMatch->nTime;
			//printf("�ɻ�%d�����Ĺ켣��", pstMatch->astWeUav[uav.num].nNO);
			/*for (int j = 0; j < uav.path3d.size(); j++) {
				printf("%d,%d,%d", uav.path3d[j].x, uav.path3d[j].y, uav.path3d[j].z);
				if (j < uav.path3d.size() - 1) {
				printf("->");
				}
				else {
				printf("\n");
				}
				}*/
		}
		else {
			if (pstMap->nMapX > 20) {
				vector<Point3d> path3d;
				for (int i = 0; i < uav.pathtogoods.size(); i++) {
					path3d.push_back(uav.pathtogoods[i]);
				}
				for (int i = 0; i < gloablestatus.goodsmap[uav.goodstotransport].pathstarttoend.size(); i++) {
					path3d.push_back(gloablestatus.goodsmap[uav.goodstotransport].pathstarttoend[i]);
				}
				uav.path3d.clear();
				uav.path3d = path3d;
				/*printf("�ɻ�%d�˻��Ĺ켣��", pstMatch->astWeUav[uav.num].nNO);
				for (int j = 0; j < uav.path3d.size(); j++) {
					printf("%d,%d,%d", uav.path3d[j].x, uav.path3d[j].y, uav.path3d[j].z);
					if (j < uav.path3d.size() - 1) {
						printf("->");
					}
					else {
						printf("\n");
					}
				}*/
			}
			else {
				vector<Point3d> path3d;
				for (int i = 0; i < uav.pathtogoods.size(); i++) {
					path3d.push_back(uav.pathtogoods[i]);
				}
				for (int i = 0; i < gloablestatus.goodsmap[uav.goodstotransport].pathstarttoend.size(); i++) {
					path3d.push_back(gloablestatus.goodsmap[uav.goodstotransport].pathstarttoend[i]);
				}
				uav.path3d.clear();
				uav.path3d = path3d;
				/*printf("�ɻ�%d�˻��Ĺ켣��", pstMatch->astWeUav[uav.num].nNO);
				for (int j = 0; j < uav.path3d.size(); j++) {
					printf("%d,%d,%d", uav.path3d[j].x, uav.path3d[j].y, uav.path3d[j].z);
					if (j < uav.path3d.size() - 1) {
						printf("->");
					}
					else {
						printf("\n");
					}
				}*/
			}
		}
	}
}

//���������ռ������·��
inline vector<Point3d> Caculateclosestpath(Point3d startpoint, Point3d endpoint, vector<vector<vector<char>>> map3d, MAP_INFO *pstMap,int workingheight) {
	//�����ͼ�Ƚϴ�,ֻ����ͷ��и߶�Ѱ��·��
	if (pstMap->nMapZ > 20) {

		Point start(startpoint.x, startpoint.y);
		Point end(endpoint.x, endpoint.y);
		vector<Point3d> path3d;
		if (start.x == end.x&&start.y == end.y) {
			for (int i = startpoint.z; i > 0; i--) {
				Point3d p3d(endpoint.x, endpoint.y, i - 1);
				path3d.push_back(p3d);
			}
		}
		else {
			Astar astar;
			astar.InitAstar(map3d[workingheight]);
			list<Point *> path = astar.GetPath(start, end, true);

			if (path.size() > 0) {
				if (startpoint.z == workingheight) {
					path.pop_front();
				}
				else if (startpoint.z > workingheight) {
					for (int i = startpoint.z; i > workingheight + 1; i--) {
						Point3d p3d(startpoint.x, startpoint.y, i - 1);
						path3d.push_back(p3d);
					}
				}
				else if (startpoint.z < workingheight) {
					for (int i = startpoint.z; i < workingheight - 1; i++) {
						Point3d p3d(startpoint.x, startpoint.y, i + 1);
						path3d.push_back(p3d);
					}
				}
				for (auto &p : path) {
					Point3d p3d(p->x, p->y, workingheight);
					path3d.push_back(p3d);
				}
				for (int i = workingheight; i > 0; i--) {
					Point3d p3d(endpoint.x, endpoint.y, i - 1);
					path3d.push_back(p3d);
				}
			}
		}
		return path3d;

	}
	//�����ͼ�Ƚ�С
	else {
		int max = startpoint.z > pstMap->nHLow ? startpoint.z : pstMap->nHLow;
		int temp1 = -1, temp2 = -1;
		int min = 10000;
		list<Point *> pathtemp = {};
		for (int i = max; i < pstMap->nHHigh + 1; i++) {
			Astar astar;
			astar.InitAstar(map3d[i]);
			Point start(startpoint.x, startpoint.y);
			Point end(endpoint.x, endpoint.y);
			list<Point *> path = astar.GetPath(start, end, true);
			temp2 = path.size() + 2 * (i - max);
			if (temp2 <= min) {
				min = temp2;
				temp1 = i;
				pathtemp = path;
			}
		}
		vector<Point3d> path3d;
		if (temp1 != -1) {
			if (pathtemp.size() > 0) {
				if (startpoint.z == temp1) {
					pathtemp.pop_front();
				}
				else if (startpoint.z > temp1) {
					for (int i = startpoint.z; i > temp1 + 1; i--) {
						Point3d p3d(startpoint.x, startpoint.y, i - 1);
						path3d.push_back(p3d);
					}
				}
				else if (startpoint.z < pstMap->nHLow) {
					for (int i = startpoint.z; i < temp1 - 1; i++) {
						Point3d p3d(startpoint.x, startpoint.y, i + 1);
						path3d.push_back(p3d);
					}
				}
				for (auto &p : pathtemp) {
					Point3d p3d(p->x, p->y, temp1);
					path3d.push_back(p3d);
				}
				for (int i = temp1; i > 0; i--) {
					Point3d p3d(endpoint.x, endpoint.y, i - 1);
					path3d.push_back(p3d);
				}
			}
			else if (startpoint.x == endpoint.x&&startpoint.y == endpoint.y) {
				for (int i = startpoint.z; i > 0; i--) {
					Point3d p3d(endpoint.x, endpoint.y, i - 1);
					path3d.push_back(p3d);
				}
			}
			return path3d;
			/*for (int i = startpoint.z; i < temp1 - 1; i++) {
				Point3d p3d(startpoint.x, startpoint.y, i + 1);
				path3d.push_back(p3d);
			}
			if (pathtemp.size() > 0) {
				if (startpoint.z == temp1) {
					pathtemp.pop_front();
				}
				for (auto &p : pathtemp) {
					Point3d p3d(p->x, p->y, temp1);
					path3d.push_back(p3d);
				}
			}
			for (int i = temp1; i > 0; i--) {
				Point3d p3d(endpoint.x, endpoint.y, i - 1);
				path3d.push_back(p3d);
			}
			return path3d;*/
		}
	}
	return {};
}
inline int Caculate3ddistance(Gloablestatus& gloablestatus,Point3d startpoint, Point3d endpoint, Point3d goodsendpoint, int &fasteststep,bool ifhigher) {
	int dx1 = abs(startpoint.x - endpoint.x);
	int dy1 = abs(startpoint.y - endpoint.y);
	int dz1 = abs(startpoint.z - endpoint.z);
	int dx2 = abs(endpoint.x - goodsendpoint.x);
	int dy2 = abs(endpoint.y - goodsendpoint.y);
	int max1 = dx1 > dy1 ? dx1 : dy1;
	int max2 = dx2 > dy2 ? dx2 : dy2;
	if (dx1 == 0 && dy1 == 0) {
		fasteststep = max1 + dz1;
		return dz1 + max1 + max2 + 2 * gloablestatus.heightofworkinguav+2;
	}
	else {
		fasteststep = max1 + 2 * gloablestatus.heightofworkinguav - dz1;
		return max1 + max2 + 4 * gloablestatus.heightofworkinguav - dz1+2+2*(ifhigher?1:0);
	}
}
//����ɻ��ӵ�ǰλ�õ����ﵽ�յ�����·��
inline bool Isgoodschoosed(int goodsnum, vector<int> goodschoosed) {
	for (int i = 0; i < goodschoosed.size(); i++) {
		if (goodsnum == goodschoosed[i]) {
			return true;
		}
	}
	return false;
}
inline bool Isenemyuavchoosed(int enemyuavnum, Gloablestatus& gloablestatue) {
	for (int i = 0; i < gloablestatue.enemyuavchoosed.size(); i++) {
		if (enemyuavnum == gloablestatue.enemyuavchoosed[i]) {
			return true;
		}
	}
	return false;
}

inline bool Isnewuavdestoryed(MATCH_STATUS * pstMatch, vector<Uav>& uav, Gloablestatus& gloablestatus) {
	int numofdestroyeduav = 0;
	for (int i = 0; i < uav.size(); i++) {
		if (pstMatch->astWeUav[uav[i].num].nStatus == 1) {
			numofdestroyeduav++;
		}
	}
	if (numofdestroyeduav > gloablestatus.numofdestroyeduav) {
		gloablestatus.numofdestroyeduav = numofdestroyeduav;
		return true;
	}
	return false;
}
inline void Setplan(vector<int> &goodschoosed, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<Uav>& uav,Gloablestatus& gloablestatus) {
	int numofuavnotdestroyed = 0;
	for (size_t i = 0; i < uav.size(); i++) {
		if (pstMatch->astWeUav[uav[i].num].nStatus != 1) {
			//if (uav[i].time_temp != -1) {
				if (uav[i].ischarging) {
					UAV uavtemp;
					uavtemp.nNO = pstMatch->astWeUav[uav[i].num].nNO;
					uavtemp.nX = pstMatch->astWeUav[uav[i].num].nX;
					uavtemp.nY = pstMatch->astWeUav[uav[i].num].nY;
					uavtemp.nZ = pstMatch->astWeUav[uav[i].num].nZ;
					uavtemp.nGoodsNo = pstMatch->astWeUav[uav[i].num].nGoodsNo;
					int remain_electricity = pstMatch->astWeUav[uav[i].num].remain_electricity + uav[i].charge;
					if (remain_electricity >= uav[i].capacity) {
						uavtemp.remain_electricity = uav[i].capacity;
						uav[i].isfullcharge = true;
					}
					else {
						uavtemp.remain_electricity = remain_electricity;
					}
					pstFlayPlane->astUav[numofuavnotdestroyed] = uavtemp;
				}
				else if (uav[i].isfree) {
					printf("feiji %d de wei zhi:%d,%d,%d\n", i, uav[i].path3d[pstMatch->nTime - uav[i].time_temp].x, uav[i].path3d[pstMatch->nTime - uav[i].time_temp].y, uav[i].path3d[pstMatch->nTime - uav[i].time_temp].z);
					//uav[i].issetpath = true;
					UAV uavtemp;
					uavtemp.nNO = pstMatch->astWeUav[uav[i].num].nNO;
					uavtemp.nX = uav[i].path3d[pstMatch->nTime - uav[i].time_temp].x;
					uavtemp.nY = uav[i].path3d[pstMatch->nTime - uav[i].time_temp].y;
					uavtemp.nZ = uav[i].path3d[pstMatch->nTime - uav[i].time_temp].z;
					if (uavtemp.nX == pstMatch->astWeUav[uav[i].num].nX&&uavtemp.nY == pstMatch->astWeUav[uav[i].num].nY&&uavtemp.nZ == pstMatch->astWeUav[uav[i].num].nZ) {
						uav[i].isstoped = true;
					}
					else {
						uav[i].isstoped = false;
					}
					uavtemp.nGoodsNo = pstMatch->astWeUav[uav[i].num].nGoodsNo;
					uavtemp.remain_electricity = pstMatch->astWeUav[uav[i].num].remain_electricity;
					pstFlayPlane->astUav[numofuavnotdestroyed] = uavtemp;
				}
				else if (pstMatch->nTime - uav[i].time_temp < uav[i].path3d.size()) {
					printf("feiji %d de wei zhi:%d,%d,%d\n", i, uav[i].path3d[pstMatch->nTime - uav[i].time_temp].x, uav[i].path3d[pstMatch->nTime - uav[i].time_temp].y, uav[i].path3d[pstMatch->nTime - uav[i].time_temp].z);
					UAV uavtemp;
					uavtemp.nNO = pstMatch->astWeUav[uav[i].num].nNO;
					uavtemp.nX = uav[i].path3d[pstMatch->nTime - uav[i].time_temp].x;
					uavtemp.nY = uav[i].path3d[pstMatch->nTime - uav[i].time_temp].y;
					uavtemp.nZ = uav[i].path3d[pstMatch->nTime - uav[i].time_temp].z;
					if (uavtemp.nX == pstMatch->astWeUav[uav[i].num].nX&&uavtemp.nY == pstMatch->astWeUav[uav[i].num].nY&&uavtemp.nZ == pstMatch->astWeUav[uav[i].num].nZ) {
						uav[i].isstoped = true;
					}
					else {
						uav[i].isstoped = false;
					}
					uavtemp.nGoodsNo = pstMatch->astWeUav[uav[i].num].nGoodsNo;
					if (uav[i].path3d[pstMatch->nTime - uav[i].time_temp].z == 0 && pstMatch->nTime - uav[i].time_temp - uav[i].path3d.size() != -1) {
						for (int k = 0; k < pstMatch->nGoodsNum; k++) {
							if (uav[i].goodstotransport == pstMatch->astGoods[k].nNO) {
								if (pstMatch->astGoods[k].nState == 0) {
									uavtemp.nGoodsNo = uav[i].goodstotransport;
									uav[i].istranspointing = true;
									uav[i].istotransporting = false;
									break;
								}
								else
								{
									uav[i].isfree = true;
									uav[i].issetpath = true;
									uav[i].istotransporting = false;
								}
							}
						}
					}
					if (uav[i].istranspointing) {
						int remain_electricity = pstMatch->astWeUav[uav[i].num].remain_electricity - gloablestatus.goodsmap[uavtemp.nGoodsNo].weight;
						if (remain_electricity > 0) {
							uavtemp.remain_electricity = remain_electricity;
						}
						else {
							uavtemp.remain_electricity = 0;
						}
						uav[i].isfullcharge = false;
					}
					else {
						uavtemp.remain_electricity = pstMatch->astWeUav[uav[i].num].remain_electricity;
					}
					if (pstMatch->nTime - uav[i].time_temp - uav[i].path3d.size() == -1) {
						if (uav[i].isbackinghome) {
							uav[i].isbackinghome = false;

							uav[i].ischarging = true;
							uav[i].isfree = false;

							int remain_electricity = pstMatch->astWeUav[uav[i].num].remain_electricity + uav[i].charge;
							if (remain_electricity >= uav[i].capacity) {
								uavtemp.remain_electricity = uav[i].capacity;
								uav[i].isfullcharge = true;
							}
							else {
								uavtemp.remain_electricity = remain_electricity;
							}
							uav[i].isfree = false;

						}
						else {
							uav[i].isfree = true;
							uav[i].issetpath = true;
							uav[i].istranspointing = false;
							uav[i].goodstotransport = -1;

							if (uav[i].isdefended) {
								uav[i].isdefended = false;
								uav[uav[i].idofdefendinguav].isdefending = false;
								uav[uav[i].idofdefendinguav].isfree = true;
								uav[uav[i].idofdefendinguav].issetpath = true;
							}
						}
					}
					pstFlayPlane->astUav[numofuavnotdestroyed] = uavtemp;
				}
			numofuavnotdestroyed++;
		}
	}
	pstFlayPlane->nUavNum = numofuavnotdestroyed;
}
inline bool Istwouavcollide(Point3d Anow, Point3d Anext, Point3d Bnow, Point3d Bnext) {
	if ((Anext.x == Bnext.x&&Anext.y == Bnext.y&&Anext.z == Bnext.z) || (Anext.x == Bnow.x&&Anext.y == Bnow.y&&Anext.z == Bnow.z&&Anow.x == Bnext.x&&Anow.y == Bnext.y&&Anow.z == Bnext.z)
		||(Anow.z==Anext.z&&Bnext.z==Bnow.z&&Anow.z==Bnow.z&&Anow.x==Bnow.x&&Anow.y==Bnext.y&&Anext.x==Bnext.x&&Anext.y==Bnow.y)
		||(Anow.z == Anext.z&&Bnext.z == Bnow.z&&Anow.z == Bnow.z&&Anow.x == Bnext.x&&Anow.y == Bnow.y&&Anext.x == Bnow.x&&Anext.y == Bnext.y)) {
		return true;
	}
	else {
		return false;
	}
}
inline void Adjustpathfortwouav(Uav& uavA, Uav& uavB, MATCH_STATUS * pstMatch, MAP_INFO *pstMap, vector<vector<vector<char>>> map3d) {

	if (uavA.isstoped) {
		if (pstMatch->astWeUav[uavB.num].nZ != pstMatch->astWeUav[uavA.num].nZ) {
			Point3d point3d(pstMatch->astWeUav[uavA.num].nX, pstMatch->astWeUav[uavA.num].nY, pstMatch->astWeUav[uavA.num].nZ);
			Point3d p3d1 = Getadjacentpoints(pstMap, pstMatch, map3d, point3d);
			Point3d p3d2=point3d;
			uavA.path3d.insert(uavA.path3d.begin() + (pstMatch->nTime - uavA.time_temp), p3d1);
			uavA.path3d.insert(uavA.path3d.begin() + (pstMatch->nTime - uavA.time_temp+1), p3d2);
		}
		else if (pstMatch->astWeUav[uavA.num].nZ + 1 <= pstMap->nHHigh) {
			Point3d p3d1(pstMatch->astWeUav[uavA.num].nX, pstMatch->astWeUav[uavA.num].nY, pstMatch->astWeUav[uavA.num].nZ + 1);
			Point3d p3d2(uavA.path3d[pstMatch->nTime - uavA.time_temp].x, uavA.path3d[pstMatch->nTime - uavA.time_temp].y, uavA.path3d[pstMatch->nTime - uavA.time_temp].z + 1);
			uavA.path3d.insert(uavA.path3d.begin() +( pstMatch->nTime - uavA.time_temp), p3d1);
			uavA.path3d.insert(uavA.path3d.begin() + (pstMatch->nTime - uavA.time_temp + 1), p3d2);
		}
	}
	else if (uavB.isstoped) {
		if (pstMatch->astWeUav[uavB.num].nZ != pstMatch->astWeUav[uavA.num].nZ) {
			Point3d point3d(pstMatch->astWeUav[uavB.num].nX, pstMatch->astWeUav[uavB.num].nY, pstMatch->astWeUav[uavB.num].nZ);
			Point3d p3d1 = Getadjacentpoints(pstMap, pstMatch, map3d, point3d);
			uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp), p3d1);
			Point3d p3d2 = point3d;
			uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp+1), p3d2);
		}
		else if (pstMatch->astWeUav[uavB.num].nZ + 1 <= pstMap->nHHigh) {
			Point3d p3d1(pstMatch->astWeUav[uavB.num].nX, pstMatch->astWeUav[uavB.num].nY, pstMatch->astWeUav[uavB.num].nZ + 1);
			Point3d p3d2(uavB.path3d[pstMatch->nTime - uavB.time_temp].x, uavB.path3d[pstMatch->nTime - uavB.time_temp].y, uavB.path3d[pstMatch->nTime - uavB.time_temp].z + 1);
			uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp), p3d1);
			uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp + 1), p3d2);
		}
	}
	else if (pstMatch->astWeUav[uavA.num].nZ > pstMatch->astWeUav[uavB.num].nZ) {
		if (pstMatch->astWeUav[uavA.num].nZ + 1 <= pstMap->nHHigh) {
			Point3d p3d1(pstMatch->astWeUav[uavA.num].nX, pstMatch->astWeUav[uavA.num].nY, pstMatch->astWeUav[uavA.num].nZ + 1);
			Point3d p3d2(uavA.path3d[pstMatch->nTime - uavA.time_temp].x, uavA.path3d[pstMatch->nTime - uavA.time_temp].y, uavA.path3d[pstMatch->nTime - uavA.time_temp].z + 1);
			uavA.path3d.insert(uavA.path3d.begin() + (pstMatch->nTime - uavA.time_temp), p3d1);
			uavA.path3d.insert(uavA.path3d.begin() + (pstMatch->nTime - uavA.time_temp + 1), p3d2);
		}
	}
	else if (pstMatch->astWeUav[uavA.num].nZ < pstMatch->astWeUav[uavB.num].nZ) {
		if (pstMatch->astWeUav[uavB.num].nZ + 1 <= pstMap->nHHigh) {
			Point3d p3d1(pstMatch->astWeUav[uavB.num].nX, pstMatch->astWeUav[uavB.num].nY, pstMatch->astWeUav[uavB.num].nZ + 1);
			Point3d p3d2(uavB.path3d[pstMatch->nTime - uavB.time_temp].x, uavB.path3d[pstMatch->nTime - uavB.time_temp].y, uavB.path3d[pstMatch->nTime - uavB.time_temp].z + 1);
			uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp), p3d1);
			uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp + 1), p3d2);
		}
	}
	else {
		if (uavA.isfree) {
			if (pstMatch->astWeUav[uavA.num].nZ + 1 <= pstMap->nHHigh) {
				Point3d p3d1(pstMatch->astWeUav[uavA.num].nX, pstMatch->astWeUav[uavA.num].nY, pstMatch->astWeUav[uavA.num].nZ + 1);
				Point3d p3d2(uavA.path3d[pstMatch->nTime - uavA.time_temp].x, uavA.path3d[pstMatch->nTime - uavA.time_temp].y, uavA.path3d[pstMatch->nTime - uavA.time_temp].z + 1);
				uavA.path3d.insert(uavA.path3d.begin() + (pstMatch->nTime - uavA.time_temp), p3d1);
				uavA.path3d.insert(uavA.path3d.begin() + (pstMatch->nTime - uavA.time_temp + 1), p3d2);
			}
		}
		else {
			if (pstMatch->astWeUav[uavB.num].nZ + 1 <= pstMap->nHHigh) {
				Point3d p3d1(pstMatch->astWeUav[uavB.num].nX, pstMatch->astWeUav[uavB.num].nY, pstMatch->astWeUav[uavB.num].nZ + 1);
				Point3d p3d2(uavB.path3d[pstMatch->nTime - uavB.time_temp].x, uavB.path3d[pstMatch->nTime - uavB.time_temp].y, uavB.path3d[pstMatch->nTime - uavB.time_temp].z + 1);
				uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp), p3d1);
				uavB.path3d.insert(uavB.path3d.begin() + (pstMatch->nTime - uavB.time_temp + 1), p3d2);
			}
		}
	}
}
inline void AlgorithmCalculationFun(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<vector<vector<char>>> map3d, int time_temp, vector<Point3d> path3d)
{
}
inline void Setnewuavinfo(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, vector<vector<vector<char>>> map3d, int numofnewuav, Gloablestatus& gloablestatus) {
	for (int i = uav.size() - numofnewuav; i < uav.size(); i++) {

		uav[i].isfree = false;
		uav[i].isfullcharge = false;
		uav[i].ischarging = true;
		uav[i].issetpath = false;
		uav[i].istranspointing = false;
		uav[i].time_temp = -1;
	}
	Setuavfreetimeposition(uav, pstMap, map3d, gloablestatus);
}
inline int Judgewhichuavtypetobuy(vector<UAV_PRICE>& uavprice, vector<int> rankeduavprice, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<Uav>& uav, Gloablestatus& gloablestatus) {
	if (uav.size() - gloablestatus.numofdestroyeduav < MINNUMOFUAVTOFIGHT) {
		return 0;
	}
	else {
		if (!gloablestatus.isgoodsweightcounted) {
			vector<int> numofgoodsateachweight;
			for (int i = 0; i < uavprice.size(); i++) {
				numofgoodsateachweight.push_back(0);
			}
			Point3d pstart(pstMap->astUav[0].nX, pstMap->astUav[0].nY, 0);
			for (int i = 0; i < pstMatch->nGoodsNum; i++) {
				Point3d pgoods(pstMatch->astGoods[i].nStartX, pstMatch->astGoods[i].nStartY, 0);
				if (Caculateclosetflydistanceoftwopoints(pstart, pgoods) + 2 * pstMap->nHLow <= pstMatch->astGoods[i].nLeftTime-UAVDELAYOUTTIME) {
					int k = 0;
					while (k < uavprice.size()) {
						if (pstMatch->astGoods[i].nWeight <= uavprice[uavprice.size() - k - 1].nLoadWeight) {
							numofgoodsateachweight[k]++;
							break;
						}
						else {
							k++;
						}
					}
				}
			}
			for (int i = 0; i < uavprice.size(); i++) {
				gloablestatus.numofgoodsateachweight.push_back(numofgoodsateachweight[i]);//���������{16��8��4��2��1}
			}

			vector<int> numofuavtoperchase;
			for (int i = 0; i < gloablestatus.numofgoodsateachweight.size(); i++) {
				if (gloablestatus.numofgoodsateachweight[i] > 0) {
					if (gloablestatus.numofgoodsateachweight[i] / NUMOFGOODSFORUAV == 0) {
						numofuavtoperchase.push_back(1);
					}
					else {
						if (gloablestatus.numofgoodsateachweight[i] % NUMOFGOODSFORUAV == 0) {
							numofuavtoperchase.push_back(gloablestatus.numofgoodsateachweight[i] / NUMOFGOODSFORUAV);
						}
						else {
							numofuavtoperchase.push_back(gloablestatus.numofgoodsateachweight[i] / NUMOFGOODSFORUAV+1);
						}
					}
				}
				else {
					numofuavtoperchase.push_back(0);
				}
			}
			vector<int> temp;
			for (int i = 0; i < numofuavtoperchase.size(); i++) {
				temp.push_back(numofuavtoperchase[i]);
			}
			temp[0] = 100;
			while (Ifquitloop(temp))
			{
				for (int i = 1; i < numofuavtoperchase.size(); i++) {
					if (temp[i] > 0) {
						gloablestatus.uavquenetoperchase.push_back(0);
						gloablestatus.uavquenetoperchase.push_back(i);
						temp[i]--;
					}
				}
			}
			gloablestatus.isgoodsweightcounted = true;
		}

		if (gloablestatus.indexofuavquenetoperchase < gloablestatus.uavquenetoperchase.size()) {
			return gloablestatus.uavquenetoperchase[gloablestatus.indexofuavquenetoperchase];
		}
		else
		{
			return 0;
		}
	}
}
inline int Caculateclosetflydistanceoftwopoints(Point3d A,Point3d B) {
	int dx = abs(A.x - B.x);
	int dy = abs(A.y - B.y);
	int dz = abs(A.z - B.z);
	int min = dx > dy ? dx : dy;
	return min + dz;
}
inline bool Ifquitloop(vector<int> temp) {
	for (int i = 1; i < temp.size(); i++) {
		if (temp[i] > 0) {
			return true;
		}
	}
	return false;
}
inline Point3d Getadjacentpoints(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d,Point3d point3d) {
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			if (point3d.x + i<pstMap->nMapX&&point3d.x + i>-1 && point3d.y + j<pstMap->nMapY&&point3d.y + j>-1) {
				if (map3d[point3d.z][point3d.x + i][point3d.y + j] != 1) {
					if (i == 0 && j == 0) {

					}
					else {
						Point3d p(point3d.x + i, point3d.y + j, point3d.z);
						return p;
					}
				}
			}
		}
	}
	Point3d p(-1, -1, -1);
	return p;
}
inline vector<Point3d> Getgoodsstarttoend(Point3d pstart,Point3d pend,MATCH_STATUS * pstMatch, Gloablestatus& gloablestatus, vector<vector<vector<char>>> map3d, MAP_INFO *pstMap) {
	if (pstMap->nMapX > 20) {
		Astar astar;
		astar.InitAstar(map3d[pstMap->nHLow+1]);
		Point start(pstart.x,pstart.y);
		Point end(pend.x,pend.y);
		list<Point *> path = astar.GetPath(start, end, true);
		vector<Point3d> path3d;
		for (int i = 0; i < pstMap->nHLow ; i++) {
			Point3d p3d(pstart.x, pstart.y, i + 1);
			path3d.push_back(p3d);
		}
		for (auto &p : path) {
			Point3d p3d(p->x, p->y, pstMap->nHLow+1);
			path3d.push_back(p3d);
		}
		for (int i = pstMap->nHLow+1; i > 0; i--) {
			Point3d p3d(pend.x, pend.y, i - 1);
			path3d.push_back(p3d);
		}
		return path3d;
	}
	else {
		int temp1 = -1, temp2 = -1;
		int min = 10000;
		list<Point *> pathtemp = {};
		for (int i = pstMap->nHLow; i < pstMap->nHHigh + 1; i++) {
			Astar astar;
			astar.InitAstar(map3d[i]);
			Point start(pstart.x, pstart.y);
			Point end(pend.x, pend.y);
			list<Point *> path = astar.GetPath(start, end, true);
			temp2 = path.size() + 2 * (i - pstMap->nHLow);
			if (temp2 < min) {
				min = temp2;
				temp1 = i;
				pathtemp = path;
			}
		}
		vector<Point3d> path3d;
		if (temp1 != -1) {
			for (int i = 0; i < temp1 - 1; i++) {
				Point3d p3d(pstart.x, pstart.y, i + 1);
				path3d.push_back(p3d);
			}
			for (auto &p : pathtemp) {
				Point3d p3d(p->x, p->y, temp1);
				path3d.push_back(p3d);
			}
			for (int i = temp1; i > 0; i--) {
				Point3d p3d(pend.x, pend.y, i - 1);
				path3d.push_back(p3d);
			}
		}
		return path3d;
	}
}
