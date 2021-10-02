#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <random>
#include <chrono>

const int height=1000;
const int width=1000;
const int step_size=40;
const int iterations=800;


// stores x,y coordinate
struct coordinate{
    int y;
    int x;
    coordinate(int x, int y){
        this->y = y;
        this->x = x;
    }
};

// stores a node in the tree
struct node{
    node * parent = nullptr;
    coordinate * Coordinate = nullptr;
    node(node * parent, struct coordinate * coordinate){
        this->parent = parent;
        this->Coordinate = coordinate;
    }
};

// rrt algorithm
class rrt
{
public:

	int start_x, start_y,goal_x,goal_y,obstacle_count,obstacle_vertices,obstacle_x,obstacle_y;
	std::vector<node*> tree;
	std::vector<std::vector<coordinate *> > obstacles;
	std::vector<node*> final_path;

	// accept start point, goal point and obstacles
	void input()
	{
		// accept and store end node
		std::cout<<"Goal point as x y : ";
		std::cin>>goal_x>>goal_y ;
		node * goal_node = new node (nullptr, nullptr);
		goal_node->Coordinate = new coordinate(goal_x, goal_y);
		tree.push_back(goal_node);

		// accept and store start node
		std::cout<<"Start point as x y : ";
		std::cin>>start_x>>start_y;
		node * start_node = new node (nullptr, nullptr);
		start_node->Coordinate = new coordinate(start_x, start_y);
		tree.push_back(start_node);

		// accept and store obstacle co-ordinates
		std::cout<<"Enter number of obstacles : ";
		std::cin>>obstacle_count;
		for (int i=0;i<obstacle_count;i++)
		{
			std::vector<coordinate *> obstacle_coordinate;
			coordinate * obst_coordinate = nullptr;
			std::cout<<"Enter number of vertices in obstacle "<<i+1<< " (more than 2): ";
			std::cin>>obstacle_vertices;
			std::cout<<"Enter x y coordinates of vertices in clockwise or anti-clockwise manner "<<std::endl;
			for (int j=1;j<=obstacle_vertices;j++)
			{
				std::cout<<"Vertex "<<j<<" : ";
				std::cin>>obstacle_x>>obstacle_y ;
				obst_coordinate = new coordinate(obstacle_x, obstacle_y);
				obstacle_coordinate.push_back(obst_coordinate);
			}
			obstacles.push_back(obstacle_coordinate);
		}
	}

	// generate random node
	node* random_node()
	{
		bool valid;
		// generate random values
		std::mt19937 sample(std::chrono::steady_clock::now().time_since_epoch().count());
		node * randomnode = new node (nullptr, nullptr);
		randomnode->Coordinate = new coordinate(sample()%width, sample()%height);

		// check whether random node is in the obstacle
		for (unsigned int i = 0; i < obstacles.size(); i++)
		{
			valid=node_validity_check(obstacles[i].size(), i, randomnode->Coordinate->x, randomnode->Coordinate->y);
			if(valid==true)
			{
				break;
			}
		}

		if(valid==false)
		{
			node* nearestnode = new node (nullptr,nullptr);
			nearestnode = nearest_node(randomnode);

			// check whether edge intersects with the obstacle
			for (unsigned int i = 0; i < obstacles.size(); i++)
			{
				valid=edge_obstacle_intersection_check(obstacles[i].size(),i,nearestnode,randomnode);
				if(valid==true)
				{
					break;
				}
			}
			if(valid==false)
			{
				return randomnode;
			}
			else
			{
				randomnode = random_node();
				return randomnode;
			}
		}
		else
		{
			randomnode = random_node();
			return randomnode;
		}

	}

	// find the nearest node to the random node
	node* nearest_node(node* random_node)
	{
		double distance,nearest;
		node * nearest_node = new node(nullptr, nullptr);
		for(unsigned int i=0; i < tree.size(); i++)
		{
			distance=sqrt(pow(tree.at(i)->Coordinate->x - random_node->Coordinate->x, 2) + pow(tree.at(i)->Coordinate->y - random_node->Coordinate->y, 2));
			if(i==1)
			{
				nearest=distance;
				nearest_node=tree.at(i);
			}
			else if(distance<nearest)
			{
				nearest=distance;
				nearest_node=tree.at(i);
			}
		}
		return nearest_node;
	}

	// find node at a distance of step_size from nearest node in the direction of random node
	node* new_node(node* nearest_node, node* random_node)
	{
		int new_x,new_y;
		double distance;
		distance=sqrt(pow(nearest_node->Coordinate->x - random_node->Coordinate->x, 2) + pow(nearest_node->Coordinate->y - random_node->Coordinate->y, 2));
		node * new_node = new node(nullptr, nullptr);
		if(step_size<distance)
		{
			new_x=nearest_node->Coordinate->x - (step_size*(nearest_node->Coordinate->x - random_node->Coordinate->x))/distance;
			new_y=nearest_node->Coordinate->y - (step_size*(nearest_node->Coordinate->y - random_node->Coordinate->y))/distance;
			new_node->Coordinate = new coordinate(new_x,new_y);
		}
		else
		{
			new_node->Coordinate = new coordinate(random_node->Coordinate->x,random_node->Coordinate->y);
		}
		return new_node;
	}

	// add new node to the tree
	void insert_node(node * nearest_node, node* new_node)
	{
		node * next_node = new node (nearest_node, nullptr);
		next_node->Coordinate = new coordinate(new_node->Coordinate->x, new_node->Coordinate->y);
		tree.push_back(next_node);
	}

	// check whether random node is in the obstacle
	bool node_validity_check(int obstacle_vertices, int number, int random_x, int random_y)
	{
	  int i, j;
	  bool flag = false;
	  for (i = 0, j = obstacle_vertices-1; i < obstacle_vertices; j = i++) {
	    if ( ((obstacles[number][i]->y>random_y) != (obstacles[number][j]->y>random_y)) &&
	     (random_x < (obstacles[number][j]->x-obstacles[number][i]->x) * (random_y-obstacles[number][i]->y) / (obstacles[number][j]->y-obstacles[number][i]->y) + obstacles[number][i]->x) )
	       flag = !flag;
	  }
	  return flag;
	}

	// determine orientation - collinear, clockwise, counterclockwise
	int get_orientation(int p_x,int p_y,int q_x,int q_y,int r_x, int r_y)
	{
		int orientation = (q_y - p_y) * (r_x - q_x) - (q_x - p_x) * (r_y - q_y);
		if (orientation == 0)
		{
			return 0;
		}
		else
		{
			return (orientation > 0)? 1: 2;
		}
	}

	// check whether edge intersects with the obstacle
	bool edge_obstacle_intersection_check(int obstacle_vertices, int number,node* nearest_node, node* random_node)
	{
		for (int i=0; i < (obstacle_vertices-1); i++)
		{
			int o1 = get_orientation(nearest_node->Coordinate->x,nearest_node->Coordinate->y, random_node->Coordinate->x,random_node->Coordinate->y, obstacles[number][i]->x,obstacles[number][i]->y);
			int o2 = get_orientation(nearest_node->Coordinate->x,nearest_node->Coordinate->y, random_node->Coordinate->x,random_node->Coordinate->y, obstacles[number][i+1]->x,obstacles[number][i+1]->y);
			int o3 = get_orientation(obstacles[number][i]->x,obstacles[number][i]->y, obstacles[number][i+1]->x,obstacles[number][i+1]->y, nearest_node->Coordinate->x,nearest_node->Coordinate->y);
			int o4 = get_orientation(obstacles[number][i]->x,obstacles[number][i]->y, obstacles[number][i+1]->x,obstacles[number][i+1]->y,random_node->Coordinate->x,random_node->Coordinate->y);

			if (o1 != o2 && o3 != o4)
			{
				return true;
			}
		}

		int o1 = get_orientation(nearest_node->Coordinate->x,nearest_node->Coordinate->y, random_node->Coordinate->x,random_node->Coordinate->y, obstacles[number][0]->x,obstacles[number][0]->y);
		int o2 = get_orientation(nearest_node->Coordinate->x,nearest_node->Coordinate->y, random_node->Coordinate->x,random_node->Coordinate->y, obstacles[number][obstacle_vertices-1]->x,obstacles[number][obstacle_vertices-1]->y);
		int o3 = get_orientation(obstacles[number][0]->x,obstacles[number][0]->y, obstacles[number][obstacle_vertices-1]->x,obstacles[number][obstacle_vertices-1]->y, nearest_node->Coordinate->x,nearest_node->Coordinate->y);
		int o4 = get_orientation(obstacles[number][0]->x,obstacles[number][0]->y, obstacles[number][obstacle_vertices-1]->x,obstacles[number][obstacle_vertices-1]->y,random_node->Coordinate->x,random_node->Coordinate->y);
		if (o1 != o2 && o3 != o4)
		{
			return true;
		}

		return false;
	}

	// check whether the goal is reached or not
	bool reached_goal(node* new_node)
	{
		double distance;
		distance=sqrt(pow(tree.at(0)->Coordinate->x - new_node->Coordinate->x, 2) + pow(tree.at(0)->Coordinate->y - new_node->Coordinate->y, 2));
		if (distance<=step_size)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// final path from start point to goal point
	void path()
	{
		node* trace_node = new node(nullptr,nullptr);
		trace_node = tree.at(0);
		while(trace_node != nullptr)
		{
			final_path.push_back(trace_node);
		    trace_node = trace_node->parent;
		}
		std::cout<<"Number of nodes required to reach the goal point : "<<final_path.size();
	}

	// display RRT algorithm using SFML
	sf::ConvexShape convex;
	sf::CircleShape circle;

	// display start point, end point and obstacles
	void draw_input(sf::RenderWindow& window)
	{
		 // goal point
		 circle.setPosition(tree.at(0)->Coordinate->x,tree.at(0)->Coordinate->y);
		 circle.setRadius(5);
		 circle.setOrigin(circle.getRadius(), circle.getRadius());
		 circle.setFillColor(sf::Color(204, 51, 255));
		 window.draw(circle);

		 // start point
		 circle.setPosition(tree.at(1)->Coordinate->x,tree.at(1)->Coordinate->y);
		 circle.setRadius(5);
		 circle.setOrigin(circle.getRadius(), circle.getRadius());
		 circle.setFillColor(sf::Color(0, 255, 0));
		 window.draw(circle);

		 //obstacles
		for (unsigned int i=0; i<obstacles.size();i++)
		{
			convex.setPointCount(obstacles[i].size());
			for (int j=0;(unsigned)j<obstacles[i].size();j++)
			{
				convex.setPoint(j, sf::Vector2f(obstacles[i][j]->x,obstacles[i][j]->y));
			}
			convex.setFillColor(sf::Color(230, 230, 255));
			window.draw(convex);
		}
	}

	// display all nodes
	void draw_nodes(sf::RenderWindow& window, node* new_node)
	{
		 circle.setPosition(new_node->Coordinate->x,new_node->Coordinate->y);
		 circle.setRadius(2);
		 circle.setOrigin(circle.getRadius(), circle.getRadius());
		 circle.setFillColor(sf::Color(0, 0, 255));
		 window.draw(circle);
	}

	// display all edges
	void draw_edges(sf::RenderWindow& window, node* new_node)
	{
		sf::Vertex line[] =
		{
			sf::Vertex(sf::Vector2f(new_node->parent->Coordinate->x, new_node->parent->Coordinate->y)),
			sf::Vertex(sf::Vector2f(new_node->Coordinate->x, new_node->Coordinate->y))
		};
		window.draw(line, 2, sf::Lines);
	}

	// display path from start to end point
	void draw_path(sf::RenderWindow& window)
	{
		for(unsigned int i=0; i < (final_path.size()-1); i++)
		{
			sf::Vertex line[] =
			{
				sf::Vertex(sf::Vector2f(final_path.at(i)->parent->Coordinate->x, final_path.at(i)->parent->Coordinate->y)),
				sf::Vertex(sf::Vector2f(final_path.at(i)->Coordinate->x, final_path.at(i)->Coordinate->y))
			};
			line[0].color= sf::Color::Red;
			line[1].color= sf::Color::Red;
			window.draw(line, 2, sf::Lines);
		}
	}
};


int main()
{
	rrt RRT;
	int flag=0;

	node* random_node = new node (nullptr,nullptr);
	node* nearest_node = new node (nullptr,nullptr);
	node* new_node = new node (nullptr,nullptr);

	RRT.input(); // accept start point, goal point and obstacles

	// perform rrt
	for(int i=0;i<iterations;i++)
	{
		random_node=RRT.random_node();
		nearest_node=RRT.nearest_node(random_node);
		new_node=RRT.new_node(nearest_node,random_node);
		RRT.insert_node(nearest_node, new_node);
		// select goal node as next node
		if(flag==0)
		{
			if(RRT.reached_goal(new_node)==true)
			{
				std::cout<<std::endl;
				std::cout<<"Reached goal point in "<<i+1<<" iterations"<<std::endl;
				RRT.tree.at(0)->parent=RRT.tree.back();
				flag=1;
			}
		}
	}

	// get final path
	RRT.path();
	std::cout<<std::endl;
	// display path nodes as (x,y) co-ordinates - console
	for(int i=(RRT.final_path.size()-1);i>=0;i--)
	{
		std::cout <<RRT.final_path.size()-i<<". ("<<RRT.final_path.at(i)->Coordinate->x<< ","<< RRT.final_path.at(i)->Coordinate->y<<")"<<std::endl;
	}

	std::cout<<std::endl;
	// display all nodes as (x,y) co-ordinates - console
	std::cout<<"All explored nodes"<<std::endl;
	for(int i=0; (unsigned)i < RRT.tree.size(); i++)
	{
		std::cout << i+1<<".  ("<<RRT.tree.at(i)->Coordinate->x<<','<<RRT.tree.at(i)->Coordinate->y<< ')'<<std::endl;
	}

	// SFML display
	sf::RenderWindow window(sf::VideoMode(width,height),"Rapidly-exploring Random Tree");

	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				window.close();
			}
		}

		window.clear();

		// display start point, end point and obstacles
		RRT.draw_input(window);

		// display all nodes and edges
		for(unsigned int i=0; i < RRT.tree.size(); i++)
		{
			if(RRT.tree.at(i)->parent==nullptr)
			{
				continue;
			}
			else
			{
				RRT.draw_nodes(window, RRT.tree.at(i));
				RRT.draw_edges(window, RRT.tree.at(i));
			}
		}

		// display path
		RRT.draw_path(window);
		window.display();
	}
	return 0;
}
