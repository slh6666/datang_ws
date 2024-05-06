//
// Created by lihao on 19-7-9.
//

#include "Astar.h"

namespace pathplanning{

void Astar::InitAstar(Mat& _Map, AstarConfig _config)
{
    Mat Mask;
    InitAstar(_Map, Mask, _config);
}

void Astar::InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config)
{
    int neighbor8[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

    Map = _Map;
    config = _config;
    neighbor = Mat(8, 2, CV_8S, neighbor8).clone();

    MapProcess(Mask);
}

void Astar::PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path)
{
    // Get variables
    //cout << "astar 1" << endl;
    startPoint = _startPoint;
    targetPoint = _targetPoint;
    //cout << "astar 1" << endl;
    // Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode, path);
}

void Astar::DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask, Scalar color,
        int thickness, Scalar maskcolor)
{
    if(path.empty())
    {
        cout << "Path is empty!" << endl;
        return;
    }
    _Map.setTo(maskcolor, Mask);
    for(auto it:path)
    {
        rectangle(_Map, it, it, color, thickness);
    }
}

void Astar::MapProcess(Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();
    //ofstream out("output.txt")
   //cout << "Map:" << _Map << "end" << endl;

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    if(config.OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, config.OccupyThresh, 255, cv::THRESH_BINARY);
    }

    // Inflate
    Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    LabelMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
             if(_Map.at<uchar>(y, x) == 0)
             {
                 LabelMap.at<uchar>(y, x) = obstacle;
             }
             else
             {
                 LabelMap.at<uchar>(y, x) = free;
             }
            //LabelMap.at<uchar>(y, x) = free;
        }
    }
}

Node* Astar::FindPath()
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(pair<int, Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;
    //cout << "astar 2" << endl;
    while(!OpenList.empty())
    {
	//cout << "astar 3.0" << endl;
        // Find the node with least F value
        Point CurPoint = OpenList.top().second;
	//cout << "astar 3.1" << endl;
        OpenList.pop();
        int index = point2index(CurPoint);
	//cout << "astar 3.2" << endl;
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);
	//cout << "astar 3.3" << endl;

        int curX = CurPoint.x;
        int curY = CurPoint.y;
	//cout << "astar 3.4" << endl;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;
	//cout << "astar 3.5" << endl;
	
        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }
	else
	{ 
	  cout << curX<<","<<curY << endl;
	}

        // Traversal the neighborhood
        //for(int k=0;k < neighbor.rows;k++)
	for(int k=0;k < 8;k++)
        {
	    int neig_test[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
            };
	    int y = curY + neig_test[k][0];
	    int x = curX + neig_test[k][1];
	    //cout << "for 1" << endl;
           // int y = curY + neighbor.at<char>(k, 0);
           // int x = curX + neighbor.at<char>(k, 1);
	    //cout << neighbor.at<char>(k, 0) <<"'"<< neighbor.at<char>(k, 1) << endl;
	    cout << x <<"'"<< y << endl;
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
		cout << "continue 1" << endl;
                continue;
            }
	    //uchar* a = _LabelMap.ptr(y);
	    //uchar a = _LabelMap.ptr(y)[x];

	    //cout << <<"a="<< a << endl;
	    //cout << _LabelMap.at<int>(y, x) << endl;
	    //cout << "free is" << free << endl;
	    //cout << "obstacle is" << obstacle << endl;
            //cout << "inOpenList is" << inOpenList << endl;
            //cout << "inOpenList<int> is" << <int>inOpenList << endl;

            /*
		
	    ///bug location///255
	
            ///bug description:
		"can't receive map information"
		"all the places in ".pgm" file has been described as "obstacle"
	    ///
			
	    */
	    cout << _LabelMap.at<uchar>(curY,curX)<<","<<_LabelMap.at<uchar>(y, x) << endl;

            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)	
 	    //if(_LabelMap.at<int>(y, x) == -1 || _LabelMap.at<uchar>(y, x) == inOpenList)	            
	    {
                // Determine whether a diagonal line can pass
		 cout << "continue 2" << endl;
                //int dist1 = abs(neighbor.at<char>(k, 0)) + abs(neighbor.at<char>(k, 1)); neig_test[k][0]
		int dist1 = abs(neig_test[k][0]) + abs(neig_test[k][1]); 
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
		{
			cout << "continue 3" << endl;                    
			continue;
		}	
                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                if(config.Euclidean)
                {
                    int dist2 = (x - targetPoint.x) * (x - targetPoint.x) + (y - targetPoint.y) * (y - targetPoint.y);
                    H = round(10 * sqrt(dist2));
                }
                else
                {
                    H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                }
                F = G + H;
 		cout << "cost 1" << endl;
                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
		    cout << "search 1" << endl;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
		    cout << "search 2" << endl;
                    int index = point2index(Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
			cout << "search 3" << endl;
                    }
                }
            }
        }
    }

    return NULL; // Can not find a valid path
}

void Astar::GetPath(Node* TailNode, vector<Point>& path)
{
    cout << "astar 4" << endl;
    PathList.clear();
    path.clear();
    
    	
    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
	//cout << "astar 5" << endl;
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to vector<Point>
    int length = PathList.size();

    
    for(int i = 0;i < length;i++)
    {
        path.push_back(PathList.back()->point);
        PathList.pop_back();
        
    }

    // Release memory
    while(OpenList.size()) {
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}

}
