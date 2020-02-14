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

	Node(PointT p, int i)
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

	void insert(PointT point, int index)
	{
		insert(root, 0, point, index);
	}

	void insert(std::shared_ptr<Node<PointT>>& parent, uint depth, PointT point, int index)
	{
		if( parent == nullptr )
		{
			parent = std::make_shared<Node<PointT>>(point, index);
		}
		else
		{
			uint dim = depth % 3;
			if (point.getArray3fMap()[dim] < parent->point.getArray3fMap()[dim])
			{
				insert(parent->left, depth+1, point, index);
			}
			else
			{
				insert(parent->right, depth+1, point, index);
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> indexes;
		search(target, root, 0, distanceTol, indexes);

		return indexes;
	}

	void search(PointT target, std::shared_ptr<Node<PointT>> node, uint depth, float distanceTol, std::vector<int>& indexes)
	{
		if(node != nullptr)
		{
            Eigen::Vector3f p_curr = node->point.getVector3fMap();
            Eigen::Vector3f p = target.getVector3fMap();
			Eigen::Array3f minPoint = node->point.getArray3fMap() - distanceTol;
            Eigen::Array3f maxPoint = node->point.getArray3fMap() + distanceTol;

			bool inCube = (minPoint < p.array()).all() && ( p.array() < maxPoint).all();
			
            if ( inCube )
			{
				float distance = (p - p_curr).norm();
				if(distance < distanceTol)
					indexes.push_back(node->index);
			}
			uint dim = depth % 3;
			if(p[dim] - distanceTol < p_curr[dim])
				search(target, node->left, depth+1, distanceTol, indexes);
			if(p[dim] + distanceTol > p_curr[dim])
				search(target, node->right, depth+1, distanceTol, indexes);
		}
	}
	

};