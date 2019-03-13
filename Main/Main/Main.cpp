/*Team Id		: <#NS 4716>
* Author List	: <Manav Chhibber>
* Filename		: <Main.cpp>
* Theme			: <Nutty Squirel>
* Functions		:<move(int&, int, int&), direction(vertex , vertex, int), dijkstra(int**, int , int , int &, int []), make_turn(int, int),
				  minDistance(int [], int []), forward_wls(unsigned char ), left_turn_wls(), right_turn_wls()>
* GlobalVariables: <wall, curr_dir, dest_dir, threshold, dist_from_node, all_nodes, graph> *
* Main Theme Implementation file
*/
unsigned char threshold=60; // used in line following
int v_left=120,v_right=130; //adjusting speed of left and right motor
int flag=0;
#define F_CPU 16000000
int z=0;
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "lcd.h"
#include "color.h"
#include "arm.h"
#include "main.h"

#define FRONT_IR_ADC_CHANNEL 4
#define n 28
#define inf 9999

static int wall = 0; //for checking wall

struct vertex //node of graph
{
	float x, y;
};

int curr_dir =0;
int dest_dir;
int curr_node = 27;
int nut_count=6;
unsigned char close=140;	//value to detect nut or obstacle by robot

void forward_wls(unsigned char node);
void left_turn_wls();
void right_turn_wls();

//moves robot from start node to goal node
void move(int &start, int goal, int &curr_dir);
//find direction of destination node from current node
int direction(vertex dest, vertex src, int d);
//make turn in direction of destination node
void make_turn(int cd, int dd);
//used in path finding algorithm to find minimum distance of node from source node
int minDistance(int dist[], int visited[]);
//path finding algorithm
void dijkstra(int graph[n][n], int src, int goal, int &z, int moves[]);

//adjency matrix representation of arena
//rows is node number(0-28) starting from top left
//nth column value in ith row is distance of nth node from ith row
int graph[n][n]={ {0,1,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{1,0,1,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,1,0,1,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,1,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,1,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{1,0,0,0,0,0,0,0,1,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,2,0,0,0,0,0,1,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,2,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
				{0,0,0,0,0,2,0,0,0,0,0,1,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,2,0,0,0,1,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,2,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,1,2,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,2,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,2,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,0,0,0,0,0,1,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,2,0},
				{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0},
				{0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, };

int nuts[6]={0,1,2,4,5,6};		//Pickup Zone
int red_deposit[]={18,21,24,26};//Depositions Zone for Red Nuts
int green_deposit[]={20,15};	//Deposition Zone for Green Nuts
int blue_deposit[]={0};			//Deposition Zone for Blue Nuts
//all nodes of graph represented with coordinates
//on arena all_nodes[0] is x: horizontal increasing towards right
//on arena all_nodes[1] is y: vertical increasing towards down
vertex all_nodes[n] = { {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{0,2},{1,2},{3,2}, {5,2},{6,2},{2,3},{4,3},{0,4},{1,4},{1.5,4},{4.5,4},{5,4},{6,4},{0,5}, {6,5},{3,5},{3,7},{1,8},{3,8},{5,8},{3,3} };


/*Function Name		: <move>
* Input				: <start(initial node no.)-> the source node, goal(final node no.)-> the destination node, curr_dir->current direction of robot>
* Output			: <void>
* Logic				: <Move function moves from robot from <start> to <goal node>, it uses dijkstra algorithm to find path of nodes and stores them in array
                       then it moves from node to node taking appropiate turns using make_turn function >
* Example Call		: <move(0,1,{1,0}>
*/
void move(int &start, int goal, int &curr_dir)
{
	int i = 0, j;
	int moves[n], z = 0;
	//path finding algorithm
	dijkstra(graph, start, goal, z, moves);
	for (j = 1; j < z; j++)
	{
		dest_dir = direction(all_nodes[moves[j]], all_nodes[start], curr_dir);//determine final direction which robot will be facing when it reaches destination node
		lcd_init();
		lcd_print(1,1,start,1);
		lcd_print(1,4,moves[j],1);
		lcd_print(1,8,curr_dir,1);
		_delay_ms(1000);
		

		make_turn(curr_dir, dest_dir);
		curr_dir = dest_dir;
		if((start==23&&moves[j]==24)||(start==24&&moves[j]==23))//for the lift to work efficiently
			_delay_ms(10000);
		forward_wls(1);
		if (wall)
		{
			graph[start][moves[j]] = 0;
			graph[moves[j]][start] = 0;
			left();
			_delay_ms(930);
			stop();
			forward_wls(1);
			curr_dir = (dest_dir+8)%16;
			return;
		}

		start = moves[j];
	}


}

/*Function Name		: <direction>
* Input				: <vertex dest->destination vertex, vertex src->initial vertex, int d>
* Output			: <int defining direction >
* Logic				: <16 directions are mapped to numbers from 0-15 e.g. North:0, North-North-East:1, North-East:2 etc.
					  This function finds direction of destination vertex from current vertex. Arena is represented by vertices given by all_nodes.
					  eg. if current vertex is {0,0} and destination is {1,0} then destination lies to right of current vertex or West by 4th case>
* Example Call		: <direction({1,0},{0,1},5)> *
*/
int direction(vertex dest, vertex src, int d)
{
	if ((dest.x - src.x) < 0)
	{
		//North-North-West
		if ((dest.x - src.x) == -0.5 && (dest.y - src.y) == -1)
		{
			d =15;
		}
		//North-West-West
		else if((dest.x - src.x) == -1 && (dest.y - src.y) == -0.5)
		{
			d =13;
		}
		//North-West
		else if ((dest.x - src.x)==-1 &&(dest.y - src.y) ==-1)
			d = 14;
		//West
		else if ((dest.y == src.y))
			d =12;
		//South-West-West
		else if ((dest.x - src.x) == -1 && (dest.y - src.y) == 0.5)
		{
			d = 11;
		}
		//South-West
		else if ((dest.x - src.x) == -1 && (dest.y - src.y) == 1)
			d = 10;
		//South-South-West
		else if ((dest.x - src.x) == -0.5 && (dest.y - src.y) == 1)
		{
			d =9;
		}
		else
		{
			if (dest.y > src.y)
				d = 10;
			else
				d = 14;
		}
	}
	else if ((dest.x - src.x) == 0)
	{
		//North
		if ((dest.y - src.y) < 0)
			d = 0;
		//South
		else
			d = 8;
	}
	else
	{
		//North-North-EasT
		if ((dest.x - src.x) == 0.5 && (dest.y - src.y) == -1)
		{
			d =1;
		}
		//North-East-East
		else if ((dest.x - src.x) == 1 && (dest.y - src.y) == -0.5)
		{
			d =3;
		}
		//North-East
		else if ((dest.x - src.x) == 1 && (dest.y - src.y) == -1)
			d = 2;
		//East
		else if ((dest.y == src.y))
			d = 4;
		//South-East-East
		else if ((dest.x - src.x) == 1 && (dest.y - src.y) == 0.5)
		{
			d = 5;
		}
		//South-East
		else if ((dest.x - src.x) == 1 && (dest.y - src.y) == 1)
			d = 6;
		//South-South-East
		else if ((dest.x - src.x) == 0.5 && (dest.y - src.y) == 1)
		{
			d = 7;
		}
		else
		{
			if (dest.y > src.y)
				d = 6;
			else
				d = 2;
		}
	}
	return d;
}


/*Function Name		: <make_turn>
* Input				: <cd-> current direction,dd-> destination direction>
* Output			: <void>
* Logic				: <Make turn in the direction of destination node. If current direction is left of destination direction then right turn has to be made
                       else if it is in right of destination direction left turn has to be made. Further if dd-cd <0 then we add 16 to apply above logic.
                       Whenever dd-cd is less than 8 we make right tur, if dd-cd>8 we make left turn other, iif its 0 then no turn since its in same direction as
                       destination and if its 8 then make reverse turn or 180 degrees turn.
					   e.g. if current direction is North(0) and destination is East(4), current direction is left of destination then dest-curr=>4-0=4
                       now 4<8 this results in right turn>
* Example Call		: <make_turn(0,4)> *
*/
void make_turn(int cd, int dd)
{
	dd =( dd - cd);
	if(dd<0)
	{
		dd = dd + 16;
	}
	cd = 0;
	if(dd==0)
	{
		return;
	}
	if (dd > 8)
	{
		left_turn_wls();
	}
	else if (dd < 8)
	{
		right_turn_wls();
	}
	else
	{
		velocity(v_left+60,v_right+60);
		right();
		_delay_ms(2750);
		stop();
		velocity(v_left,v_right);
	}
}

/*Function Name	: <minDistance>
* Input			: <dist[]->array of distances from source to every node , visited[]-> array of nodes visited (whose distance from source can't be changed), values are passed by refrence >
* Output		: <min_index-> index of the node which is at minimum distance from source.>
* Logic			: <Used in path finding dijsktra algorithm to find minimum distance of every node from source.
                   Minimum found distance of every node from source is stored in dist[] array, everytime dijkstra's algorithm run it finds the node which is at minimum distance
                   from the source. This is because we require path to be shortest and dijkstra's uses the minimum distance node to proceed. We find the shortest distance node
                   which is visited.>
* Example Call	: <minDistance(dist[],visited[])> *
*/

int minDistance(int dist[], int visited[])
{
	int min = inf, min_index;

	for (int v = 0; v < n; v++)
	{
		if (visited[v] == 0 && dist[v] <= min)
		{
			min = dist[v];
			min_index = v;
		}
	}
	return min_index;
}

/*Function Name	: <dijkstra>
* Input			: <graph-> representation of arena, src-> Source node, goal-> Goal node, z-> no. of nodes to transversed in path,
                   moves-> Array containing list of nodes or the final path of nodes which has to be covered by robot>
* Output		: <None>
* Logic			: <It uses Dijkstra's Algorithm to find minimum distances of every node from source node keeping track of nodes visited and assigning parent to every node,
                   if goal node is encountered it returns. Then path is stored in moves[] by finding parent of every node. List of nodes depicting shortest path are stored in moves[] array
				  >
* Example Call: <dijkstra(int graph[n][n], int src, int goal, int &z, int moves[])> *
*/

void dijkstra(int graph[n][n], int src, int goal, int &z, int moves[])
{
	int i, u, v;
	z = 1;
	int visited[n], parent[n], dist[n];
	//initialize every node to infinity distance from each other and visited array containing zero nodes
	for (i = 0; i < n; i++)
	{
		dist[i] = inf;
		visited[i] = 0;
	}
	//distance of source to source is zero
	dist[src] = 0;
	visited[src] = 0;
	//parent of source is set to -1
	parent[src] = -1;
	for (i = 0; i < n; i++)
	{
		//find node minimum distance from current node
		//u=0 for first run
		if (i == 0)
			u = src;
		else
			u = minDistance(dist, visited);
		visited[u] = 1;
		//if goal is reached returns
		if (u == goal)
		{
			//used to find path
			while (parent[u] != -1)
			{
				u = parent[u];
				z++;
			}
			u = goal;
			//stores path in moves[]
			for (int k = 0; k < z&&u != -1; k++)
			{
				moves[z - k - 1] = u;
				u = parent[u];
			}
			return;
		}
		//if distance of the node is less than previous stored distance value than replace
		for (v = 0; v < n; v++)
		{
			if (!visited[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v])
			{

				parent[v] = u;
				dist[v] = dist[u] + graph[u][v];
			}
		}

	}

}


/*
*
* Function Name	: left_turn_wls
* Input			: void
* Output		: void
* Logic			: Uses white line sensors to turn left until black line is encountered
* Example Call	: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(void)
{
	unsigned char center_sensor;
	left();
	velocity(v_left, v_right);
	//turn away from current direction slightly
	_delay_ms(500);
	while (1)
	{		
		//checks if middle sensor comes on black line
		if ((ADC_Conversion(2) >threshold)&&ADC_Conversion(1)<threshold&&ADC_Conversion(3)<threshold)
			break;
	}
	stop();
}

/*
*
* Function Name	: right_turn_wls
* Input			: void
* Output		: void
* Logic			: Uses white line sensors to turn right until black line is encountered
* Example Call	: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(void)
{
	
	right();
	velocity(v_left, v_right);
	//turn away from current direction slightly
	_delay_ms(800);
	while (1)
	{
		//checks if middle sensor comes on black line
		if ((ADC_Conversion(2) >threshold)&&ADC_Conversion(1)<threshold&&ADC_Conversion(3)<threshold)
		{
			break;
		}		
	}
	stop();
}





/*Function Name		: <forward_wls>
* Input				: <no. of nodes to move>
* Output			: <void>
* Logic				: <Move to number of nodes specified>
* Example Call		: <forward_wls(1)>
*/

	
void forward_wls(unsigned char node)
{
	flag=0;
	unsigned char left_sensor=ADC_Conversion(1), center_sensor=ADC_Conversion(2), right_sensor=ADC_Conversion(3);
	
	print_sensor(2,1,1);
	print_sensor(2,5,2);
	print_sensor(2,9,3);
	if(ADC_Conversion(FRONT_IR_ADC_CHANNEL)>close)
	{
		wall=1;
		stop();
		return;
	}
	//runs upto given node value
	if (left_sensor >threshold && center_sensor >threshold && right_sensor> threshold)
	{
		forward();

		_delay_ms(350);
		stop();
	}
	forward();
	velocity(v_left,v_right);
	while (node)
	{
			left_sensor = ADC_Conversion(1);
			center_sensor = ADC_Conversion(2);
			right_sensor = ADC_Conversion(3);
			
			if(ADC_Conversion(FRONT_IR_ADC_CHANNEL)>close)
			{
				wall=1;
				stop();
				lcd_init();
				lcd_string("OBSTACLE");
				return;
			}
			if(center_sensor>threshold)
			{
				forward();
				if(left_sensor<threshold&&right_sensor<threshold)
				{
					velocity(v_left,v_right);
				}
				else if(left_sensor>threshold&&right_sensor<threshold)
				{
					velocity(v_left-40,v_right+40);
				}
				else if(right_sensor>threshold&&left_sensor<threshold)
				{
					velocity(v_left+40,v_right-40);
				}
				else
				{
					lcd_print(1,1,left_sensor,3);
					lcd_print(1,5,center_sensor,3);
					lcd_print(1,9,right_sensor,3);
					node--;
					stop();
					velocity(v_left,v_right);
					forward();
					if(z==1)
					_delay_ms(800);
					else 
					_delay_ms(1100);
					stop();
					continue;
				}
			}
			else
			{
				
				if(left_sensor>threshold)
				{
					flag=1;
					velocity(0,v_right);
					
				}
				else if(right_sensor>threshold)
				{
					flag=3;
					velocity(v_left,0);
				}
				else
				{
					if(flag==1)
					{
						velocity(0,v_right);
						
					}					
					else if(flag==3)
					{
						velocity(v_left,0);
					}
				}
			}
			

	}

}


int main()
{
	int i,nut=0;
	int red_nut_cnt=0,green_nut_cnt=0,blue_nut_cnt=0;
	
	init_devices();
	servo_1_free();
	servo_2_free();
	lcd_init();
	lcd_set_4bit();
	


	
	for (i = 0; i < nut_count; i++)
	{
		//move from current position to nut position
		move(curr_node, nuts[i], curr_dir);
		//checks for wall
		while (wall)
		{
			wall = 0;
			move(curr_node, nuts[i], curr_dir);
		}
		make_turn(curr_dir, 0);
		curr_dir = 0;
		stop();
		adjust_pick();
		//checks if nut is present
		if(ADC_Conversion(FRONT_IR_ADC_CHANNEL)<close)
		{
			servo_1(153);
			continue;
		}
		
		
		nut=glow();
		
		if (nut==2)	//if its green nut
		{
			pick();
			curr_node = nuts[i];
			curr_dir = 0;
			//move to destination
			move(curr_node, green_deposit[green_nut_cnt], curr_dir);
			//if obstacle encountered
			while (wall)
			{
				wall = 0;
				move(curr_node,green_deposit[green_nut_cnt], curr_dir);
			}
			make_turn(curr_dir,8);
			curr_dir=8;
			
			adjust_place();
			
			place();
			green_nut_cnt++;
		}
		else if (nut==1)	//if it is red nut
		{
			pick();
			//move to destination
			move(curr_node, red_deposit[red_nut_cnt], curr_dir);
			while (wall)
			{
				wall = 0;
				move(curr_node,red_deposit[red_nut_cnt] , curr_dir);
			}
			make_turn(curr_dir,8);
			curr_dir = 8;
			adjust_place();
			
			place();
			red_nut_cnt++;
		}
		else if(nut==3)//if its blue nut
		{
			pick();
			//move to destination
			move(curr_node, blue_deposit[blue_nut_cnt], curr_dir);
			while (wall)
			{
				wall = 0;
				move(curr_node, blue_deposit[blue_nut_cnt], curr_dir);
			}
			make_turn(curr_dir,8);
			curr_dir = 8;
			adjust_place();
			place();
			blue_nut_cnt++;
		}

	}
	buzzer_on();


}