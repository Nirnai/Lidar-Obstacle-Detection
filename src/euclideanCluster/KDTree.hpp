#pragma once

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
    int index;
	std::shared_ptr<Node<PointT>> left;
	std::shared_ptr<Node<PointT>> right;

	Node(const PointT& p, int i)
	:point(p), index(i), left(nullptr), right(nullptr)
	{}
};

template<typename PointT>
struct KdTree
{
	std::shared_ptr<Node<PointT>> root;

	KdTree()
	:root(nullptr)
	{}

	void insert(const PointT& point, int index)
	{
		insert(root, point, index, 0);
	}

	void insert(std::shared_ptr<Node<PointT>>& parent, const PointT& point, int index, int depth)
	{
		if( parent == nullptr )
		{
			parent = std::make_shared<Node<PointT>>(point, index);
		}
		else
		{
			int dim = depth % 3;
			if (point.getArray3fMap()[dim] < parent->point.getArray3fMap()[dim])
			{
				insert(parent->left, point, index, depth+1);
			}
			else
			{
				insert(parent->right, point, index, depth+1);
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const PointT& target, float distanceTol)
	{
		std::vector<int> indexes;
		search(target, root, 0, distanceTol, indexes);

		return indexes;
	}

	void search(const PointT& p, std::shared_ptr<Node<PointT>> node, uint depth, float distanceTol, std::vector<int>& indexes)
	{
		if(node != nullptr)
		{

			bool inCube = p.x < (node->point.x + distanceTol) &&  
						  p.y < (node->point.y + distanceTol) && 
						  p.z < (node->point.z + distanceTol) &&
						  p.x > (node->point.x - distanceTol) &&
						  p.y > (node->point.y - distanceTol) &&
						  p.z > (node->point.z - distanceTol);

			if(inCube)
			{
				float distance = std::sqrt(std::pow((node->point.x - p.x),2) + std::pow((node->point.y - p.y),2) + std::pow((node->point.z - p.z),2));
				if(distance < distanceTol)
					indexes.push_back(node->index);
			}
			uint dim = depth % 3;
			if(p.data[dim] - distanceTol < node->point.data[dim])
				search(p, node->left, depth+1, distanceTol, indexes);
			if(p.data[dim] + distanceTol > node->point.data[dim])
				search(p, node->right, depth+1, distanceTol, indexes);
			// Eigen::Vector3f p_curr  = node->point.getVector3fMap(); 
			// Eigen::Vector3f p       = target.getVector3fMap();
			// Eigen::Array3f minPoint = node->point.getArray3fMap() - distanceTol;
			// Eigen::Array3f maxPoint = node->point.getArray3fMap() + distanceTol;

			// bool inCube = (minPoint < p.array()).all() && ( p.array() < maxPoint).all();
            // if ( inCube )
			// {
			// 	float distance = 1;//(p - p_curr).norm();
			// 	if(distance < distanceTol)
			// 		indexes.push_back(node->index);
			// }


			// uint dim = depth % 3;
			// if(p[dim] - distanceTol < p_curr[dim])
			// 	search(p, node->left, depth+1, distanceTol, indexes);
			// if(p[dim] + distanceTol > p_curr[dim])
			// 	search(p, node->right, depth+1, distanceTol, indexes);
		}
	}
	

};