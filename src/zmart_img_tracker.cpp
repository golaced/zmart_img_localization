#include "zmart_img/zmart_img_tracker.h"

using namespace std;

CvPoint2D64f FindLineCross(ZmartImgLine line1, ZmartImgLine line2)
{
    CvPoint2D64f point;
    vector<double> Arow1, Arow2, Crow1, Crow2;

    Arow1.push_back(line1.A);
	Arow1.push_back(line1.B);
	Arow2.push_back(line2.A);
	Arow2.push_back(line2.B);
	Crow1.push_back(-line1.C);
	Crow2.push_back(-line2.C);

	Matrix A,B,C;
	A.push_back(Arow1);
	A.push_back(Arow2);
	C.push_back(Crow1);
	C.push_back(Crow2);

	B = inverse(A)*C;

	point.x = B[0][0];
	point.y = B[1][0];

	return point;
	//figure out the crosspoint coordinate of line1 and line2
}

ZmartImgTracker::ZmartImgTracker(void)
{
    initialized = false;
    frame_offset_vertical = 0;
    frame_offset_horizontal = 0;

    translation_vector = Mat_<double>(3,1);
    rotation_vector = Mat_<double>(3,1);
    translation_vector = 0;
    rotation_vector = 0; //initialize
}

ZmartImgTracker::~ZmartImgTracker(void)
{

}

bool ZmartImgTracker::IsInitialized()
{
    return initialized;
}


bool ZmartImgTracker::Initialize(vector<ZmartImgLine> horizontal_raw, vector<ZmartImgLine> vertical_raw)
{
    LoadGrids(horizontal_raw, vertical_raw);
    //get the horizontal and vertical

    if(FindOffSet())// how to judge FindOffSet()????
    {
        horizontal_last = horizontal;//defined in the h files
        vertical_last = vertical;
        initialized = true;
        return true;
    }
    else
    {
        return false;
    }
}

void ZmartImgTracker::LoadGrids(vector<ZmartImgLine> horizontal_raw, vector<ZmartImgLine> vertical_raw)
{
    horizontal.clear(); // defined in the h files, form as vector<ZmartImgGridline>
    vertical.clear();
    for (int i =0; i<horizontal_raw.size(); i++)
    {
        horizontal.push_back(ZmartImgGridline(horizontal_raw.at(i)));
        //ZmartImgGridline(horizontal_raw.at(i)):get the No.i line's A,B,C,d,theta,length,point1,point2

    }
     for (int i=0; i< horizontal.size(); i++)
     {
        if (i-1 >=0)
        {
            horizontal.at(i).toPre(horizontal.at(i-1),HORIZONTAL);
            //since i>=1, void ZmartImgGridline::toPre(ZmartImgLine zmartimgline, bool dir)
        }
        else
        {
            horizontal.at(i).toPreNull();
        }

        if(i+1<horizontal.size())
        {
            horizontal.at(i).toPost(horizontal.at(i+1),HORIZONTAL);
        }
        else
        {
            horizontal.at(i).toPostNull();
        }

     }


     for(int i=0; i<vertical_raw.size(); i++)
		{
			vertical.push_back(ZmartImgGridline(vertical_raw.at(i)));
		}
		for(int i=0; i<vertical.size(); i++)
		{
			if(i-1>0)
			{
				vertical.at(i).toPre(vertical.at(i-1),VERTICAL);
			}
			else
			{
				vertical.at(i).toPreNull();
			}

			if(i+1<vertical.size())
			{
				vertical.at(i).toPost(vertical.at(i+1),VERTICAL);
			}
			else
			{
				vertical.at(i).toPostNull();
			}
		}


}

void ZmartImgTracker::ProcessFrame(vector<ZmartImgLine> horizontal_raw, vector<ZmartImgLine> vertical_raw)
{
    if(!initialized)//if initialized is false
    {
        Initialize(horizontal_raw,vertical_raw);
        return;
    }
    LoadGrids(horizontal_raw, vertical_raw);
    //get horizontal and vertical
    if(!FindOffset())// FindOffset() is false
    {
        frame_offset_h.clear();
        frame_offset_v.clear();
        squares.clear();

        initialized = false;
        return;
    }
    vector<vector<ZmartImgGridline> > neibour;

    for (int i = 0; i < horizontal.size(); i++)
    {
        ZmartImgGridline target = horizontal.at(i);
        vector<ZmartImgGridline> cuurent_arrange;
        current_arrange = horizontal_last;

        for (int j = 0; j < cuurent_arrange.size(); j++)
        {
            for (int k = 0; k < (cuurent_arrange.size()-1); K++)
            {
                double d_1 = current_arrange.at(k).toGridLine(target, HORIZONTAL);
                double d_2 = cuurent_arrange.at(k+1).toGridLine(target, HORIZONTAL);
                if (d_1 > d_2)
                {
                    swap(current_arrange.at(k),current_arrange.at(k+1)); //jiao huan
                }
            }
        }
        neibour.push_back(current_arrange);
    }
    frame_offset_h.clear();

    for(int i = 0; i < neibour.size(); i++)
    {
        if (abs(horizontal.at(i).toGridLine(neibour.at(i).at(0),HORIZONTAL))<60)
        {
            frame_offset_h.push_back(horizontal.at(i).world_lable - neibour.at(i).at(0).world_lable);
        }
    }
    frame_offset_horizontal = FindOffsetMode(frame_offset_h);


    neibour.clear();

    for (int i = 0; i < vertical.size(); i++)
    {
        ZmartImgGridline target = vertical.at(i);
        vector<ZmartImgGridline> current_arrange;
        current_arrange = vertical_last;
        for(int j = 0; j < current_arrange.size(); j++)
        {
            for (int k = 0; k < (current_arrange.size()-1); k++)
            {
                double d_1 = current_arrange.at(k).toGridLine(target, VERTICAL);
                double d_2 = current_arrange.at(k+1).toGridLine(target,VERTICAL);
                if(d_1 > d_2)
                {
                    swap(current_arrange.at(k), current_arrange(k+1));
                }
            }
        }
        neibour.push_back(current_arrange);
    }
    frame_offset_v.clear();

    for(int i=0; i<neibour.size();i++)
	{
		if(abs(vertical.at(i).toGridLine(neibour.at(i).at(0),VERTICAL))<60)
			frame_offset_v.push_back(vertical.at(i).world_lable - neibour.at(i).at(0).world_lable);
	}

	frame_offset_vertical = FindOffsetMode(frame_offset_v);


	for (int i = 0; i <horizontal.size(); i++)
	{
        horizontal.at(i).world_lable -=frame_offset_horizontal;
	}
	for(int i=0; i<vertical.size();i++)
	{
		vertical.at(i).world_lable -= frame_offset_vertical;
	}

	for(int i=0; i<horizontal.size();i++)
	{
		horizontal.at(i).stability = 0.5;
	}
	vector<d_unit> shift_h;
	for(int i=0; i<horizontal.size();i++)
	{
		for(int j=0;j<horizontal_last.size();j++)
		{
			if(horizontal.at(i).world_lable == horizontal_last.at(j).world_lable)
			{
				horizontal.at(i).stability+=0.4;
				shift_h.push_back(d_unit(i,horizontal.at(i).toGridLine(horizontal_last.at(j),HORIZONTAL)));
			}
		}
	}
	double average_shift=0;
	for(int i=0;i<shift_h.size();i++)
	{
		average_shift += abs(shift_h.at(i).d);
	}
	average_shift = average_shift/shift_h.size();
	for(int i=0;i<shift_h.size();i++)
	{
		double shift_rate = abs((shift_h.at(i).d - average_shift) / (10*average_shift));
		horizontal.at(shift_h.at(i).num).stability = (horizontal.at(shift_h.at(i).num).stability - shift_rate)*horizontal.at(shift_h.at(i).num).confidence_level;
	}


	for(int i=0; i<vertical.size();i++)
	{
		vertical.at(i).stability = 0.5;
	}
	vector<d_unit> shift_v;
	for(int i=0; i<vertical.size();i++)
	{
		for(int j=0;j<vertical_last.size();j++)
		{
			if(vertical.at(i).world_lable == vertical_last.at(j).world_lable)
			{
				vertical.at(i).stability+=0.4;
				shift_v.push_back(d_unit(i,vertical.at(i).toGridLine(vertical_last.at(j),HORIZONTAL)));
			}
		}
	}

	average_shift=0;
	for(int i=0;i<shift_v.size();i++)
	{
		average_shift += abs(shift_v.at(i).d);
	}
	average_shift = average_shift/shift_v.size();
	for(int i=0;i<shift_v.size();i++)
	{
		double shift_rate = abs((shift_v.at(i).d - average_shift) / (10*average_shift));
		vertical.at(shift_v.at(i).num).stability = (vertical.at(shift_v.at(i).num).stability - shift_rate)*vertical.at(shift_v.at(i).num).confidence_level;
	}

	squares.clear();
	int square_count=0;


}


bool ZmartImgTracker::FindOffSet()
{
    double min_horizontal_d = 500;
    double min_vertical_d = 500;
    double average_d = 0;
    int average_d_count = 0;

    for (int i = 0; i < horizontal.size(); i++)
    {
        if (horizontal.at(i).d_to_post > 0)
        {
            if ((horizontal.at(i).d_to_post < min_horizontal_d) && (horizontal.at(i).d_to_post > 80))
                min_horizontal_d = horizontal.at(i).d_to_post;
        }
    }

    for (int i = 0; i < vertical.size(); i++)
    {
        if(vertical.at(i).d_to_post > 0)
        {
            if ((vertical.at(i).d_to_post < min_vertical_d) && (vertical.at(i).d_to_post > 80))
                min_vertical_d = vertical.at(i).d_to_post;
        }
    }

    for (int i = 0; i < horizontal.size(); i++)
    {
        if (horizontal.at(i).d_to_post > 0)
        {
            if((horizontal.at(i).d_to_post / min_horizontal_d) < 1.8)
            {
                average_d += horizontal.at(i).d_to_post;
                average_d_count ++;
            }

        }
    }

    for(int i=0; i<vertical.size(); i++)
	{
		if(vertical.at(i).d_to_post>0)
		{
			//if(vertical[i].d_to_post<min_vertical_d)
			//	min_vertical_d = vertical[i].d_to_post;
			if((vertical.at(i).d_to_post / min_horizontal_d ) < 1.8)
			{
				average_d += vertical.at(i).d_to_post;
                average_d_count++;
			}
		}
	}

    if (horizontal.size() < 2 || vertical.size() < 2)
    {
        return false;
        cout << "Lack of detected grids" << endl;
    }

    average_d = average_d /average_d_count;


    d_upper_threshold = 1.8 * average_d;
    d_lower_threshold = 60;

    int middle = (int) horizontal.size()/2 + 0.5;// the middle line

    if (horizontal.size() >= 2)
    {
        horizontal.at(middle).world_lable = 0;
        //world_lable, offset, confidence_level are defined in zmart_img_gridline.h file
        horizontal.at(middle).offset = 0;
        horizontal.at(middle).confidence_level = 1;
    }

    for (int i = 1; (i + middle) < horizontal.size(); i++)
    {
        if((horizontal.at(middle+i).d_to_pre > d_lower_threshold) && horizontal.at(middle+i).d_to_pre < d_upper_threshold)
        {
            horizontal.at(middle+i).offset = 1;
            horizontal.at(middle+i).world_lable = horizontal.at(middle+i-1).world_lable + horizontal.at(middle+i).offset;
            horizontal.at(middle+i).confidence_level = 1 -0.1*i;
        }
        else
        {
            double mul = horizontal.at(middle+i).d_to_pre / average_d;
            if ((abs)((int)(mul+0.5) - mul) > 0.4) && (abs((int)(mul + 0.5 ) - mul) < 0.6)
            {
                int count = 0;
                for (vector<ZmartImgGridline>::iterator iter = horizontal.begin(); iter != horizontal.end(); iter++)
                {
                    if(count == middle + i)
                    {
                        horizontal.erase(iter);
                        break; // run out of while/for
                    }
                    count ++;
                    //erase horizontal.at(middle+i)
                }
            }
            else
            {
                horizontal.at(middle + i).offset = (int) (mul + 0.5);

            }
        }
    }
}

int ZmartImgTracker::FindOffsetMode(vector<int> offset)
{
    vector<int> temp;
    if(offset.size()<1)
        return false;
    for (int i = 0; i < offset.size(); i++)
    {
        temp.push_back(0);
    }
    for (int i = 0; i < temp.size(); i++)
    {
        for(int j = 0; j < temp.size(); j++)
        {
            if(offset.at(i) == offset.at(j))
            {
                temp.at(i)++;
            }

        }
    }
    int max = -1000;
    int count = -1;
    for (int i = 0; i < temp.size(); i++)
    {
        if(temp.at(i)> max)
        {
            max = temp.at(i);
            count = i;
        }
    }
    return offset.at(count);
}















