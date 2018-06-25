
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

#define MAX_SOCKET_BUFFER       (1024 * 1024 * 4)       /// 发送接受数据最大4M
#define UAVNUM_EACHLINE 4                              //地图上每一横排停放的空闲飞机数量
#define FREEUAVPATHBUFF 1000                           //空闲飞机飞行路径的后面部分长度
#define STARTFINDINGGOODS_TIME    10                   //飞机升空到等待运货时间
#define UAVTOGOODSSTEP_UP         100                  //飞机到货物的步数的最大可承受值
#define UAVNUMTOTRANSPORTLOWVALUE 15                   //当飞机数量超过这个时可以去运送小价值货物
#define RATIOOFGOODSVALUE         2                    //前期飞机运送的货物的最低价值系数
#define NUMOFGOODSFORUAV          2                    //给每家飞机分配的货物数
#define MINNUMOFUAVTOFIGHT        14                   //前期作战的飞机数量
#define MINNUMOFUAVWORKING        50                   //所有飞机数量最大值
#define UAVDELAYOUTTIME           40                   //预定的飞机到生产出来延迟的时间
using namespace std;

const int kCost1 = 10; //直移一格消耗
const int kCost2 = 10; //斜移一格消耗

struct Point
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	int F, G, H; //F=G+H
	Point *parent; //parent的坐标，这里没有用指针，从而简化代码
	Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化
	{
	}
};
struct Point3d
{
	int x, y, z; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列

	Point3d(int _x, int _y, int _z) :x(_x), y(_y), z(_z) //变量初始化
	{
	}
	Point3d() {}
};

struct Uav
{
	int num;//飞机编号
	bool isfree;//是否飞机是空闲状态的
	bool issetpath;//是否给飞机设置路径
	int time_temp;//飞机路径的时间基点
	Point3d goodsposition;//飞机要运送货物的起点
	Point3d targetposition;//飞机要运送货物的终点
	Point3d freetimeposition;//空闲时候停放的位置
	Point3d fightingpoint;//拦截敌机的地点
	Point3d defendingpoint;
	Point3d defendingpoint1;
	vector<Point3d> path3d;//飞机的飞行路径
	vector<Point3d> pathstarttoend;//货物起点到终点的路径
	vector<Point3d> pathtogoods;//飞机到货物的路径
	int goodstotransport;//飞机运送货物的编号
	bool isstoped;//是否飞机停在原处
	bool isfighting;//是否飞机要进行战斗
	bool istranspointing;//是否在运送货物
	int loadweight1;//最大载重量
	int loadweight2;//合理的载重量下限
	int goodsweight;//运送的货物的重量
	int goodsvalue;//运送的货物的价值
	int goodsrelevatevalue;//货物的相对价值
	int remainelectricity;//飞机剩余电量
	bool ischarging;//飞机是否在充电
	bool isfullcharge;//飞机是否充满电
	int capacity;//电池容量
	int charge;//单位时间充电量
	bool isdefending;//是否在防卫
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
	bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
	Point *isInList(const std::list<Point *> &list, const Point *point) const; //判断开启/关闭列表中是否包含某点
	Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
							 //计算FGH值
	int calcG(Point *temp_start, Point *point);
	int calcH(Point *point, Point *end);
	int calcF(Point *point);
private:
	std::vector<std::vector<char>> maze;
	std::list<Point *> openList;  //开启列表
	std::list<Point *> closeList; //关闭列表
};
struct Gloablestatus
{
	int numofdestroyeduav;//撞毁的飞机数量
	vector<Point3d> freepointsvector;//所有的飞机空闲时停放地点
	int maxfreepointsindex;//最大的停放点数量
	bool isgoodsweightcounted;//是否统计过不同重量货物的数量
	vector<int> numofgoodsateachweight;//处于每个重量级别的货物数量
	vector<int> uavquenetoperchase;//要购买飞机的次序向量
	int indexofuavquenetoperchase;//要购买飞机的次序向量的下标
	vector<int> enemyuavchoosed;
	int heightofworkinguav;//飞机工作高度
	Point3d enemypoint;//敌方停机坪位置
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
	int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空
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
	openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
									   //1,找到当前周围八个格中可以通过的格子
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
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
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
		}
	}

	return NULL;
}

std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//返回路径，如果没找到路径，返回空链表
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
	return path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list)
		if (p->x == point->x&&p->y == point->y)
			return p;
	return NULL;
}

bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{
	//printf("下标x,y：%d,%d\n", target->x, target->y);
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
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以
			return true;
		else
		{
			//斜对角要判断是否绊住
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
 *  @brief	接受数据
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
		// 接受头部长度
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
		// 说明数据还没接受完
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
 *  @brief	发送数据
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
 *  @brief	学生的算法计算， 参数什么的都自己写，
 *	@return void
 */
void  AlgorithmCalculationFun(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<vector<vector<char>>> map3d, int time_temp, vector<Point3d> path3d);

void printinformation(MAP_INFO *pstMap)
{
	printf("最低飞行高度: %d\n", pstMap->nHLow);
	printf("最高飞行高度: %d\n", pstMap->nHHigh);
	printf("地图长度: %d\n", pstMap->nMapX);
	printf("地图宽度u: %d\n", pstMap->nMapY);
	printf("地图高度: %d\n", pstMap->nMapZ);
	printf("parkingx： %d\n", pstMap->nParkingX);
	printf("parkingy： %d\n", pstMap->nParkingY);
	printf("buildingnum： %d\n", pstMap->nBuildingNum);
	printf("forgnum： %d\n", pstMap->nFogNum);
	printf("uavnum： %d\n", pstMap->nUavNum);
	printf("uavpricenum： %d\n", pstMap->nUavPriceNum);
	for (int i = 0; i < pstMap->nBuildingNum; i++) {
		printf("building height: %d\n", (pstMap->astBuilding)[i].nH);
	}
	printf("障碍物起始位置：%d,%d\n", pstMap->astBuilding[0].nX, pstMap->astBuilding[0].nY);
	printf("障碍物长度和宽度和高度：%d,%d,%d\n", pstMap->astBuilding[0].nL, pstMap->astBuilding[0].nW, pstMap->astBuilding[0].nH);
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

	//rankeduavprice和uavprice都是按价格从高到低排列
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
	// windows下，需要进行初始化操作
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
	 //本地调试去掉这个
	if (argc != 4)
	{
		printf("error arg num\n");
		return -1;
	}
	strcpy(szIp, argv[1]);
	nPort = atoi(argv[2]);
	strcpy(szToken, argv[3]);
	printf("server ip %s, prot %d, token %s\n", szIp, nPort, szToken);
	// 开始连接服务器
	nRet = OSCreateSocket(szIp, (unsigned int)nPort, &hSocket);
	if (nRet != 0)
	{
		printf("connect server error\n");
		return nRet;
	}
	// 分配接受发送内存
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

	// 接受数据  连接成功后，Judger会返回一条消息：
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}
	// json 解析
	// 获取头部
	CONNECT_NOTICE  stNotice;
	nRet = ParserConnect(pRecvBuffer + SOCKET_HEAD_LEN, &stNotice);
	if (nRet != 0)
	{
		return nRet;
	}
	// 生成表明身份的json
	TOKEN_INFO  stToken;
	strcpy(stToken.szToken, szToken);  // 如果是调试阶段，请输入你调试的token，在我的对战中获取，
								   // 实际比赛，不要传入调试的，按照demo写的，有服务器调用传入。
	strcpy(stToken.szAction, "sendtoken");

	memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = CreateTokenInfo(&stToken, pSendBuffer, &nLen);
	if (nRet != 0)
	{
		return nRet;
	}
	// 选手向裁判服务器表明身份(Player -> Judger)
	nRet = SendJuderData(hSocket, pSendBuffer, nLen);
	if (nRet != 0)
	{
		return nRet;
	}
	//身份验证结果(Judger -> Player)　
	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}
	// 解析验证结果的json
	TOKEN_RESULT      stResult;
	nRet = ParserTokenResult(pRecvBuffer + SOCKET_HEAD_LEN, &stResult);
	if (nRet != 0)
	{
		return 0;
	}
	// 是否验证成功
	if (stResult.nResult != 0)
	{
		printf("token check error\n");
		return -1;
	}
	// 选手向裁判服务器表明自己已准备就绪(Player -> Judger)
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
	//对战开始通知(Judger -> Player)　
	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}
	// 解析数据
	//Mapinfo 结构体可能很大，不太适合放在栈中，可以定义为全局或者内存分配
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
	// 第一次把无人机的初始赋值给flayplane
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
	attractmap3d(pstMapInfo, map3d);//获取3D地图
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
		// 进行当前时刻的数据计算, 填充飞行计划结构体，注意：0时刻不能进行移动，即第一次进入该循环时
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
			//获取地图信息
			printf("获取货物信息！\n");
			Getgoodsinfo(pstMatchStatus, gloablestatus, map3d, pstMapInfo);
			Getenemyinfo(pstMatchStatus, gloablestatus);
			Oneuavflyeachtimestamp(uav,gloablestatus,pstMatchStatus);
			//给所有飞机分配货物以及要去拦截的敌机，并且规划路径
			printf("给所有飞机分配目标！\n");
			Assigntarget(uavprice,gloablestatus,pstMapInfo, pstMatchStatus, uav, map3d, goods, goodschoosed, rankeduavprice);
			//判断是否有碰撞的情况发生
			printf("减少碰撞！\n");
			Judgecrashandadjustpath(pstMapInfo, pstMatchStatus, map3d, uav);
			//将设置好的路径等信息发送给服务器
			printf("添加飞行计划！\n");
			Addpathtoflyplan(goodschoosed, pstMapInfo, pstMatchStatus, pstFlayPlane, map3d, uav, gloablestatus);
			//购买新的飞机并且初始化这些飞机
			printf("购买飞机并进行初始化！\n");
			Buyuavandinitial(uavprice,rankeduavprice, pstMapInfo, pstMatchStatus, uav, map3d, pstFlayPlane, gloablestatus);
		}
		//发送飞行计划结构体
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
		// 接受当前比赛状态
		memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
		nRet = RecvJuderData(hSocket, pRecvBuffer);
		if (nRet != 0)
		{
			return nRet;
		}
		// 解析
		nRet = ParserMatchStatus(pRecvBuffer + SOCKET_HEAD_LEN, pstMatchStatus);
		if (nRet != 0)
		{
			return nRet;
		}
		//printf("%s\n", pRecvBuffer);

		if (pstMatchStatus->nMacthStatus == 1)
		{
			// 比赛结束
			printf("game over, we value %d, enemy value %d\n", pstMatchStatus->nWeValue, pstMatchStatus->nEnemyValue);
			return 0;
		}
	}
	// 关闭socket
	OSCloseSocket(hSocket);
	// 资源回收
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
//在地图上找一些点作为飞机自由停放时的位置，将地图横竖平均分成16份，每横每竖都是4，将飞机分散到这16个区域中的随机地点
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
		printf("购买飞机%d,型号：%s\n", uav.size(), uavprice[uavprice.size() - 1 - index].szType);
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
	printf("系统时间:%d\n", pstMatch->nTime);
	printf("pastmatch中，货物的数量%d\n", pstMatch->nGoodsNum);

	for (int i = 0; i < pstMatch->nGoodsNum; i++) {
		//pstMatch->astGoods[i].ischoosed = false;
		//printf("pstmatch中，货物的编号%d\n", pstMatch->astGoods[i].nNO);
		//printf("pstmatch中，货物剩余时间%d\n", pstMatch->astGoods[i].nLeftTime);
		//printf("货物的开始结束位置：%d,%d,%d->%d,%d,%d\n", pstMatch->astGoods[i].nStartX, pstMatch->astGoods[i].nStartY, 0, pstMatch->astGoods[i].nEndX, pstMatch->astGoods[i].nEndY, 0);
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
				printf("我方飞机%d的目标敌机%d已经撞毁!\n", uav[i].num, uav[i].idofenemyuavtofight);
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

	//安排飞机战斗
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
	printf("安排飞机战斗！\n");
	Assignuavtofight(uavprice, gloablestatus, freeuav, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//防卫自己的飞机
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
	printf("防卫自己的飞机！\n");
	Assignuavtodefence(uavprice, gloablestatus, freeuav3, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//给飞机分配货物
	vector<Uav> freeuav1;
	for (size_t i = 0; i < uav.size(); i++) {
		if (uav[i].isfree && (pstMatch->astWeUav[uav[i].num].nStatus != 1)) {
			freeuav1.push_back(uav[i]);
		}
	}
	sort(freeuav1.begin(), freeuav1.end(), [](const Uav& uav1, const Uav& uav2) {return uav1.loadweight1 < uav2.loadweight1; });
	if (pstMatch->nTime > 5) {
		printf("给飞机分配货物！\n");
		Assignuavtogoods(uavprice, gloablestatus, freeuav1, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);
	}

	//给飞机调整货物
	vector<Uav> freeuav2;
	if (pstMatch->nTime >= 6) {
		for (size_t i = 0; i < uav.size(); i++) {
			if (uav[i].istotransporting && (pstMatch->astWeUav[uav[i].num].nStatus != 1)&&pstMatch->nTime-uav[i].time_temp>0) {
				freeuav2.push_back(uav[i]);
			}
		}
	}
	printf("给飞机调整货物！\n");
	Adjustuavtogoods(uavprice, gloablestatus, freeuav2, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//保护要去运送货物的飞机
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
	printf("保护要去运送货物的飞机！\n");
	Assignuavtoprotectuavtotransport(uavprice, gloablestatus, freeuav5, pstMatch, map3d, uav, goods, pstMap, goodschoosed, rankeduavprice);

	//给飞机充电
	vector<Uav> freeuav4;
	for (int i = 0; i < uav.size(); i++) {
		if (uav[i].isfree&&pstMatch->astWeUav[uav[i].num].nStatus != 1 && !uav[i].isfullcharge&&uav[i].value > uavprice[uavprice.size() - 1].nValue
			&&pstMatch->astWeUav[uav[i].num].nZ>=pstMap->nHLow&&!uav[i].isbackinghome) {
			freeuav4.push_back(uav[i]);
		}
	}
	printf("给飞机充电！\n");
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
	printf("给所有飞机设置路径！\n");
	for (size_t i = 0; i < uav.size(); i++) {
		if (uav[i].issetpath) {
			Setpath(gloablestatus,map3d, uav[i], pstMatch, pstMap);
			uav[i].issetpath = false;
		}
	}
	return;
}
void Addpathtoflyplan(vector<int> &goodschoosed, MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, vector<vector<vector<char>>> map3d, vector<Uav>& uav, Gloablestatus& gloablestatus) {
	//对每一架飞机，添加飞行计划
	/*for (int i = 0; i < uav.size(); i++) {
		printf("飞机%d的价值：%s\n", pstMatch->astWeUav[uav[i].num].nNO, pstMatch->astWeUav[uav[i].num].szType);
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
	printf("防止碰撞！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！\n");
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
	//去撞它们
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
						printf("战斗机%d进入战斗，飞机的当前位置:%d,%d,%d，目标敌机:%d，敌机位置:%d,%d,%d，目标货物位置:%d,%d,%d\n", freeuav[i].num,pwuav.x,pwuav.y,pwuav.z, pstMatch->astEnemyUav[j].nNO,peuav.x,peuav.y,peuav.z, pgoods.x, pgoods.y, pgoods.z);
						if (uav[freeuav[i].num].goodstotransport != -1) {
							uav[freeuav[i].num].goodstotransport = -1;
							vector<int>::iterator it = goodschoosed.begin();
							for (; it != goodschoosed.end();)
							{
								if (*it == uav[freeuav[i].num].goodstotransport)
									//删除指定元素，返回指向删除元素的下一个元素的位置的迭代器
									it = goodschoosed.erase(it);
								else
									//迭代器指向下一个元素位置
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

//对于所有出现的货物，给它们分配飞机去取它们
inline void Assignuavtogoods(vector<UAV_PRICE>& uavprice,Gloablestatus& gloablestatus,vector<Uav>& freeuav, MATCH_STATUS * pstMatch, vector<vector<vector<char>>> map3d, vector<Uav>& uav, vector<Goods>& goods, MAP_INFO *pstMap, vector<int>& goodschoosed, vector<int> rankeduavprice) {

	printf("给所有飞机分配货物!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	vector<int> temp1, temp2,temp3,temp5,temp6;//临时变量，分别用来存储货物对应飞机的步长，货物对应飞机的编号，以及飞机是否被选中了的标志
	vector<vector<Point3d>> temp4;
	vector<vector<int>> costmatrix;

	//给每一架空闲的飞机寻找货物
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
				//printf("给飞机%d寻找货物\n", pstMatch->astWeUav[freeuav[i].num].nNO);
				for (int j = 0; j < pstMatch->nGoodsNum; j++) {
					if (!Isgoodschoosed(pstMatch->astGoods[j].nNO, goodschoosed)) {
						if (pstMatch->astGoods[j].nWeight <=freeuav[i].loadweight1 ) {
							Point3d startpoint(pstMatch->astWeUav[freeuav[i].num].nX, pstMatch->astWeUav[freeuav[i].num].nY, pstMatch->astWeUav[freeuav[i].num].nZ);//飞机当前坐标
							Point3d endpoint(pstMatch->astGoods[j].nStartX, pstMatch->astGoods[j].nStartY, 0);//货物起点坐标
							Point3d goodsendpoint(pstMatch->astGoods[j].nEndX, pstMatch->astGoods[j].nEndX, 0);//货物终点坐标
							int fasteststep;//飞机到货物最快步数，不考虑障碍物的情况
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
										temp2[i] = j;//表示第i架飞机对应第j个货物
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
		//对于每一个飞机设置他的状态以及要去取的货物
		for (int i = 0; i < freeuav.size(); i++) {
			if (temp2[i] != -1) {
				printf("飞机%d对应的货物：%d\n", freeuav[i].num, pstMatch->astGoods[temp2[i]].nNO);
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
				Point3d startpoint(pstMatch->astWeUav[freeuav[i].num].nX, pstMatch->astWeUav[freeuav[i].num].nY, pstMatch->astWeUav[freeuav[i].num].nZ);//飞机当前坐标
				Point3d endpoint(pstMatch->astGoods[j].nStartX, pstMatch->astGoods[j].nStartY, 0);//货物起点坐标
				Point3d goodsendpoint(pstMatch->astGoods[j].nEndX, pstMatch->astGoods[j].nEndY, 0);//货物终点坐标
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
							//删除指定元素，返回指向删除元素的下一个元素的位置的迭代器
							it = goodschoosed.erase(it);
						else
							//迭代器指向下一个元素位置
							++it;
					}
					goodschoosed.push_back(pstMatch->astGoods[j].nNO);
					uav[freeuav[i].num].goodstotransport = pstMatch->astGoods[j].nNO;
					printf("给飞机%d重新分配货物%d\n", uav[freeuav[i].num].num, pstMatch->astGoods[j].nNO);
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
				printf("僚机%d进入防卫，保护货物目标点!\n",uav[freeuav[temp1[i]].num].num);
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
				printf("僚机%d进入防卫，保护货物出发点!\n", uav[freeuav[temp1[i]].num].num);
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
		printf("飞机%d回家\n",uav[freeuav[i].num].num);
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
			printf("飞机%d回家的轨迹：", pstMatch->astWeUav[uav.num].nNO);
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

			printf("飞机%d自由飞行的轨迹：", pstMatch->astWeUav[uav.num].nNO);
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
		if (uav.isfighting) {//开始规划战斗路径，到达战斗地点并且保持不动

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
				printf("飞机%d战斗的轨迹：", pstMatch->astWeUav[uav.num].nNO);
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

				printf("飞机%d防卫的轨迹：", pstMatch->astWeUav[uav.num].nNO);
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
			//printf("飞机%d防卫的轨迹：", pstMatch->astWeUav[uav.num].nNO);
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
				/*printf("飞机%d运货的轨迹：", pstMatch->astWeUav[uav.num].nNO);
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
				/*printf("飞机%d运货的轨迹：", pstMatch->astWeUav[uav.num].nNO);
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

//计算两个空间点的最短路径
inline vector<Point3d> Caculateclosestpath(Point3d startpoint, Point3d endpoint, vector<vector<vector<char>>> map3d, MAP_INFO *pstMap,int workingheight) {
	//如果地图比较大,只在最低飞行高度寻找路径
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
	//如果地图比较小
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
//计算飞机从当前位置到货物到终点的最短路径
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
				gloablestatus.numofgoodsateachweight.push_back(numofgoodsateachweight[i]);//可能情况：{16，8，4，2，1}
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
