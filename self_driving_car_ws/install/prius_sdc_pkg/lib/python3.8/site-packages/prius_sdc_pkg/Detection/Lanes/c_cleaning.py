import cv2
import numpy as np
from .utilities import Distance, Cord_Sort


def IsPathCrossingMid(Midlane,Mid_cnts,Outer_cnts):

	is_Ref_to_path_Left = 0
	Ref_To_Path_Image = np.zeros_like(Midlane)
	Midlane_copy = Midlane.copy()

	Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
	Outer_cnts_Rowsorted = Cord_Sort(Outer_cnts,"rows")

	Mid_Rows = Mid_cnts_Rowsorted.shape[0]
	Outer_Rows = Outer_cnts_Rowsorted.shape[0]

	Mid_lowP = Mid_cnts_Rowsorted[Mid_Rows-1,:]
	Outer_lowP = Outer_cnts_Rowsorted[Outer_Rows-1,:]

	Traj_lowP = ( int( (Mid_lowP[0] + Outer_lowP[0]  ) / 2 ) , int( (Mid_lowP[1]  + Outer_lowP[1] ) / 2 ) )
	
	#cv2.line(Ref_To_Path_Image,Traj_lowP,(int(Ref_To_Path_Image.shape[1]/2),Traj_lowP[1]),(255,255,0),2)# distance of car center with lane path
	#cv2.line(Ref_To_Path_Image,(Traj_lowP[0],Ref_To_Path_Image.shape[0]),(int(Ref_To_Path_Image.shape[1]/2),Ref_To_Path_Image.shape[0]),(255,255,0),2)# distance of car center with lane path
	cv2.line(Ref_To_Path_Image,Traj_lowP,(int(Ref_To_Path_Image.shape[1]/2),Ref_To_Path_Image.shape[0]),(255,255,0),2)# distance of car center with lane path
	cv2.line(Midlane_copy,tuple(Mid_lowP),(Mid_lowP[0],Midlane_copy.shape[0]-1),(255,255,0),2)# distance of car center with lane path

	is_Ref_to_path_Left = ( (int(Ref_To_Path_Image.shape[1]/2) - Traj_lowP[0]) > 0 )
	#Distance_And_Midlane = cv2.bitwise_and(Ref_To_Path_Image,Midlane_copy)

	if( np.any( (cv2.bitwise_and(Ref_To_Path_Image,Midlane_copy) > 0) ) ):
		# Midlane and CarPath Intersets (MidCrossing)
		return True,is_Ref_to_path_Left
	else:
		return False,is_Ref_to_path_Left




def GetYellowInnerEdge(OuterLanes, MidLane, OuterLane_Points):
    Offset_correction = 0
    Outer_lanes_ret = np.zeros_like(OuterLanes.shape,OuterLanes.dtype)
    # 1. Extracting Mid and OuterLand Contours

    Mid_cnts = cv2.findContours(MidLane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    Outer_cnts = cv2.findContours(OuterLanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    # 2. Checking if OuterLane was Present initally or not
    if not Outer_cnts:
        NoOuterLane_before=True
    else:
        NoOuterLane_before=False
    
    # 3. Setting the first contour of Midlane as Reference

    Ref = (0,0) # If MidContours are present use the first ContourPoint as Ref To Find Nearest Yell
    if(Mid_cnts):
        Ref = tuple(Mid_cnts[0][0][0])

    
    # 4. >>>>>>>>>>>>>>>>>>>>>>>>>>>Condition 1 : if Both Midlane and Outlane is detected <<<<<<<<<<<<<<<<
    if Mid_cnts and (len(OuterLane_Points)==2):
        # 4.A               **[len(OuterLane_Points)==2)]**
        # (a) Fetching side of outlane nearest to midlane
        if (len(OuterLane_Points)==2):
            Point_a = OuterLane_Points[0]
            Point_b = OuterLane_Points[1]

            Closest_Index = 0
            if Distance(Point_a, Ref) <= Distance(Point_b,Ref):
                Closest_Index=0
            elif(len(Outer_cnts)>1):
                Closest_Index=1

            Outer_Lanes_ret = cv2.drawContours(Outer_lanes_ret, Outer_cnts, Closest_Index, 255,1)
            Outer_cnts_ret = [Outer_cnts[Closest_Index]]

		# ================================ Checking IF Correct Side outlane is detected =====================================
		# The idea is to find lane points here and determine if trajectory is crossing midlane
		#If (Yes):
		# Discard
		#Else 
		# Continue

		# 4. [len(OuterLane_Points)==2)] _ B: Find Connection between Mid And Detected OuterLane Crosses Mid
            IsPathCrossing , IsCrossingLeft = IsPathCrossingMid(MidLane,Mid_cnts,Outer_cnts_ret)
            if(IsPathCrossing):
               
                OuterLanes = np.zeros_like(OuterLanes)#Empty outerLane
            else:
                #If no fllor crossing return results
                return Outer_Lanes_ret ,Outer_cnts_ret, Mid_cnts,0
            
        # 4. [len(OuterLane_Points)!=2)]
        elif( Mid_cnts and np.any(OuterLanes>0) ):
            # 4. [len(OuterLane_Points)!=2)] : Checking IF Correct Side outlane is detected
            IsPathCrossing , IsCrossingLeft = IsPathCrossingMid(MidLane,Mid_cnts,Outer_cnts)
            if(IsPathCrossing):
                
                OuterLanes = np.zeros_like(OuterLanes)#Empty outerLane
            else:
               
                #If no fllor crossing return results
                return OuterLanes ,Outer_cnts, Mid_cnts,0		
            


        # 4. >>>>>>>>>>>>>> Condition 2 : if MidLane is present but no Outlane detected >>>>>>>>>>>>>> Or Outlane got zerod because of crossings Midlane
        # Action: Create Outlane on Side that represent the larger Lane as seen by camera
        if( Mid_cnts and ( not np.any(OuterLanes>0) ) ):	
            
            # Condition where MidCnts are detected 
            Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
            Mid_Rows = Mid_cnts_Rowsorted.shape[0]
            Mid_lowP = Mid_cnts_Rowsorted[Mid_Rows-1,:]
            Mid_highP = Mid_cnts_Rowsorted[0,:]
            Mid_low_Col = Mid_lowP[0]

            DrawRight = False

            # 4. [Midlane But , No OuterLanes!!!]

            # 4. [Midlane But , No OuterLanes!!!] _ A : Check if Present before or Not 
            if NoOuterLane_before:
                
                if(Mid_low_Col < int(MidLane.shape[1]/2)): # MidLane on left side of Col/2 of image --> Bigger side is right side draw there
                    DrawRight = True
            # If Outerlane was present before and got EKIA: >>> DrawRight because it was Crossing LEFt
            else:
                
                if IsCrossingLeft: # trajectory from reflane to lane path is crossing midlane while moving left --> Draw Right
                    DrawRight = True


            #Offset Correction wil be set here to correct for the yellow lane not found 
            # IF we are drawing right then  we need to correct car to move right to find that outerlane
            # Else Move Left

            # 4. [Midlane But , No OuterLanes!!!] _ D : Calculate Offset Correction
            if not DrawRight:
                low_Col=0
                high_Col=0
                Offset_correction = -20
            else:
                low_Col=(int(MidLane.shape[1])-1)
                high_Col=(int(MidLane.shape[1])-1)
                Offset_correction = 20

            Mid_lowP[1] = MidLane.shape[0]# setting mid_trajectory_lowestPoint_Row to MaxRows of Image

            LanePoint_lower =  (low_Col , int( Mid_lowP[1] ) )
            LanePoint_top   =  (high_Col, int( Mid_highP[1]) )

            # 4. [Midlane But , No OuterLanes!!!] _ B : Draw OuterLAnes according to midlane information
            OuterLanes = cv2.line(OuterLanes,LanePoint_lower,LanePoint_top,255,1)	

            # 4. [Midlane But , No OuterLanes!!!] _ C : Find OuterLane Contours	
            Outer_cnts = cv2.findContours(OuterLanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

            return OuterLanes, Outer_cnts, Mid_cnts, Offset_correction
            
        # 5. Condition 3 [No MidLane]
        else:
            return OuterLanes, Outer_cnts, Mid_cnts, Offset_correction
        




def ExtendSHortLane(MidLane,Mid_cnts, Outer_cnts, OuterLane):
         
	# 1. Sorting the Mid and Outer Contours on basis of rows (Ascending)
	if(Mid_cnts and Outer_cnts):		
		Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
		Outer_cnts_Rowsorted = Cord_Sort(Outer_cnts,"rows")

		Image_bottom = MidLane.shape[0]
		
		Lane_Rows = Mid_cnts_Rowsorted.shape[0]
		Lane_Cols = Mid_cnts_Rowsorted.shape[1]
		BottomPoint_Mid = Mid_cnts_Rowsorted[Lane_Rows-1,:]	
		
		# 2. Connect Midlane to imagebottom by drawing a Vertical line
		if (BottomPoint_Mid[1] < Image_bottom):
			MidLane = cv2.line(MidLane,tuple(BottomPoint_Mid),(BottomPoint_Mid[0],Image_bottom),255)


		RefLane_Rows = Outer_cnts_Rowsorted.shape[0]
		RefLane_Cols = Outer_cnts_Rowsorted.shape[1]
		BottomPoint_Outer = Outer_cnts_Rowsorted[RefLane_Rows-1,:]

		# 3. Connect Outerlane to imagebottom by performing 2 steps if neccasary
		if (BottomPoint_Outer[1] < Image_bottom):
			if(RefLane_Rows>20):
				shift=20
			else:
				shift=2
			RefLast10Points = Outer_cnts_Rowsorted[RefLane_Rows-shift:RefLane_Rows-1:2,:]

			# 3a. Connect Outerlane to imagebottom by Estimating its sloping and extending in
			#     the direction of that slope	
			if(len(RefLast10Points)>1):# Atleast 2 points needed to estimate a line
				Ref_x = RefLast10Points[:,0]#cols
				Ref_y = RefLast10Points[:,1]#rows
				Ref_parameters = np.polyfit(Ref_x, Ref_y, 1)
				Ref_slope = Ref_parameters[0]
				Ref_yiCntercept = Ref_parameters[1]
				#Decreasing slope means Current lane is left lane and by going towards 0 x we touchdown
				if(Ref_slope < 0):
					Ref_LineTouchPoint_col = 0
					Ref_LineTouchPoint_row = Ref_yiCntercept
				else:
					Ref_LineTouchPoint_col = OuterLane.shape[1]-1 # Cols have lenth of ColLength But traversal is from 0 to ColLength-1
					Ref_LineTouchPoint_row = Ref_slope * Ref_LineTouchPoint_col + Ref_yiCntercept
				Ref_TouchPoint = (Ref_LineTouchPoint_col,int(Ref_LineTouchPoint_row))#(col ,row)
				Ref_BottomPoint_tup = tuple(BottomPoint_Outer)
				OuterLane = cv2.line(OuterLane,Ref_TouchPoint,Ref_BottomPoint_tup,255)
				
				# 3b. Incase extended outerlane is still less then image bottom extend by
				#     drawing a vertical line
				if(Ref_LineTouchPoint_row < Image_bottom):
					Ref_TouchPoint_Ref = (Ref_LineTouchPoint_col,Image_bottom)
					OuterLane = cv2.line(OuterLane,Ref_TouchPoint,Ref_TouchPoint_Ref,255)
                              
		cv2.imshow("[ExtendShortLane] OuterLanes",OuterLane)
	else:
		cv2.destroyWindow("[ExtendShortLane] OuterLanes")
	return MidLane,OuterLane