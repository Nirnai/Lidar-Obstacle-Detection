#include <memory>
#include <cmath>
#include <algorithm>
#include <functional>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::shared_ptr<Node> left;
	std::shared_ptr<Node> right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	std::shared_ptr<Node> root;

	KdTree()
	: root(nullptr)
	{}

	void insert(std::vector<float> point, int id)
	{
		insert(root, 0, point, id);
	}

	void insert(std::shared_ptr<Node>& parent, uint depth, std::vector<float> point, int id)
	{
		if( parent == nullptr )
		{
			parent = std::make_shared<Node>(point, id);
		}
		else
		{
			uint dim = depth % point.size();
			if (point[dim] < parent->point[dim])
			{
				insert(parent->left, depth+1, point, id);
			}
			else
			{
				insert(parent->right, depth+1, point, id);
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(target, root, 0, distanceTol, ids);

		return ids;
	}

	void search(std::vector<float> target, std::shared_ptr<Node> node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != nullptr)
		{
			std::vector<float> diff;
			std::transform(node->point.begin(), node->point.end(), target.begin(),
							std::back_inserter(diff),
							std::minus<float>());
			bool inBox = (diff[0] + distanceTol) * (diff[0] - distanceTol) <= 0 && (diff[1] + distanceTol) * (diff[1] - distanceTol) <= 0;
			if ( inBox )
			{
				float distance = 0;
				for (auto x : diff)
				{
					distance += x*x;
				}
				distance = sqrt(distance);
				if(distance < distanceTol)
					ids.push_back(node->id);
			}
			uint dim = depth % target.size();
			if(target[dim] - distanceTol < node->point[dim])
				search(target, node->left, depth+1, distanceTol, ids);
			if(target[dim] + distanceTol > node->point[dim])
				search(target, node->right, depth+1, distanceTol, ids);
		}
	}
	

};