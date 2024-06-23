"""
Solar Gators Suspension Kinematics Solver
By: Quang Pham, 2024

The goal of this script is to calculate the suspension movement due to bump 
and rebound and calculate suspension parameters from a quarter-car model.

Note: +x is rearward, +y is outward (from right wheel), +z is upward. Origin: x_0 = FG_x, y_0 = center of car, z_0 = 0

See:
Mike Blundell and Damien Harty, The Multibody Systems Approach to Vehicle Dynamics. Elsevier, 2004.
Chapter 2 describes how to simulate suspension motion. Chapter 4 describes how to calculate suspension parameters.
"""
import numpy as np
import matplotlib.pyplot as plt

class kinSolve():
    def __init__(self, coords, isPlot=True):
        """
        Define the static configuration of the suspension.
        Input:
            coords: array of coordinates for each of the suspension points below for the right wheel.
                FG: initial wheel center (x,y,z)
                SP_in: spring chassis attachment
                SP_out: spring rocker attachment
                R1: rocker axis 1
                R2: rocker axis 2
                PR_in: push rod rocker attachment
                PR_out: push rod knuckle attachment
                TR_in: tie rod inboard attachment
                TR_out: tie rod upright attachment
                SA: spindle axis (x,y,z)
                UB: upper ball joint (x,y,z)
                LB: lower ball joint (x,y,z)
                WB: wheel base/contact patch (x,y,z)
                UCF: upper front (x,y,z)
                UCR: upper rear (x,y,z)
                LCF: lower front (x,y,z)
                LCR: lower rear (x,y,z)
        ===================================================================================
        Output: camber, toe, caster, mechanical trail, kpi, scrub, spring length, (optional: spring force, wheel rate)
                Graphs of all of the above vs bump motion
        """
        # Get initial coordinates
        self.FG = coords[0]
        self.SP_in = coords[1]
        self.SP_out = coords[2]
        self.R1 = coords[3]
        self.R2 = coords[4]
        self.PR_in = coords[5]
        self.PR_out = coords[6]
        self.TR_in = coords[7]
        self.TR_out = coords[8]
        self.WC = self.FG
        self.SA = coords[9]
        self.UB = coords[10]
        self.LB = coords[11]
        self.WB = coords[12]
        self.UCF = coords[13]
        self.UCR = coords[14]
        self.LCF = coords[15]
        self.LCR = coords[16]

        # Set intial lengths
        self.lR1_SP_out = np.linalg.norm(self.SP_out-self.R1) # distance from rocker axis 1 to spring rocker attachment
        self.lR2_SP_out = np.linalg.norm(self.SP_out-self.R2) # distance from rocker axis 1 to spring rocker attachment
        self.lSP_in_SP_out = np.linalg.norm(self.SP_out-self.SP_in) # initial spring length

        self.lR1_PR_in = np.linalg.norm(self.PR_in-self.R1) # distance from rocker axis 1 to push rod rocker attachment
        self.lR2_PR_in = np.linalg.norm(self.PR_in-self.R2) # distance from rocker axis 2 to push rod rocker attachment
        self.lSP_out_PR_in = np.linalg.norm(self.PR_in-self.SP_out) # distance from spring rocker attachment to push rod rocker attachment

        self.lPR_in_PR_out = np.linalg.norm(self.PR_out-self.PR_in) # length of push rod
        self.lLCF_PR_out = np.linalg.norm(self.PR_out-self.LCF) # distance from LCF to push rod knuckle attachment
        self.lLCR_PR_out = np.linalg.norm(self.PR_out-self.LCR) # distance from LCR to push rod knuckle attachment

        self.lPR_out_LB = np.linalg.norm(self.LB-self.PR_out) # distance from push rod knuckle attachment to LB
        self.lLCF_LB = np.linalg.norm(self.LB-self.LCF) # distance from LCF to LB
        self.lLCR_LB = np.linalg.norm(self.LB-self.LCR) # distance from LCR to LB

        self.lLB_UB = np.linalg.norm(self.UB-self.LB) # Distance from LB to UB
        self.lUCR_UB = np.linalg.norm(self.UB-self.UCR) # Distance from UCR to UB
        self.lUCF_UB = np.linalg.norm(self.UB-self.UCF) # Distance from UCF to UB

        self.lTR_in_TR_out = np.linalg.norm(self.TR_out-self.TR_in) # Length of tie rod
        self.lUB_TR_out = np.linalg.norm(self.TR_out-self.UB) # Distance from UB to tie rod upright attachment
        self.lLB_TR_out = np.linalg.norm(self.TR_out-self.LB) # Distance from LB to tie rod upright attachment

        self.lTR_out_SA = np.linalg.norm(self.SA-self.TR_out) # Distance from tie rod upright attachment to spindle axis
        self.lUB_SA = np.linalg.norm(self.SA-self.UB) # Distance from UB to spindle axis
        self.lLB_SA = np.linalg.norm(self.SA-self.LB) # Distance from LB to spindle axis

        self.lTR_out_WC = np.linalg.norm(self.WC-self.TR_out) # Distance from tie rod upright attachment to wheel center
        self.lUB_WC = np.linalg.norm(self.WC-self.UB) # Distance from UB to wheel center
        self.lLB_WC = np.linalg.norm(self.WC-self.LB) # Distance from LB to wheel center

        self.lUB_WB = np.linalg.norm(self.WB-self.UB) # Distance from UB to contact patch
        self.lLB_WB = np.linalg.norm(self.WB-self.LB) # Distance from LB to contact patch
        self.lWC_WB = np.linalg.norm(self.WB-self.WC) # Distance from wheel center to contact patch

        # Array of spring lengths
        self.spring = np.arange(self.lSP_in_SP_out*1.25, self.lSP_in_SP_out*0.55, -0.05)

        # Allocate arrays of suspension parameters for plotting
        self.BM_list = np.zeros_like(self.spring)
        self.HTC_list = np.zeros_like(self.spring)
        self.WR_list = np.zeros_like(self.spring)
        self.gamma_list = np.zeros_like(self.spring)
        self.delta_list = np.zeros_like(self.spring)
        self.phi_list = np.zeros_like(self.spring)
        self.TR_list = np.zeros_like(self.spring)
        self.theta_list = np.zeros_like(self.spring)
        self.GO_list = np.zeros_like(self.spring)
        self.RZ_list = np.zeros_like(self.spring)

        self.isPlot = isPlot # plotting bool

    def trilateration(self, P1, P2, P3, r1, r2, r3, P4_last):
        """
        Trilateration algorithm.
        Input: Coordinates of 3 points, known lengths from those points to a 4th point, last position of 4th point
        Output: Coordinates of a 4th point from 2 possibilities
        =====================================================================================================================
        See: https://en.wikipedia.org/wiki/True-range_multilateration#Three_Cartesian_dimensions,_three_measured_slant_ranges
        """

        # P1 is origin on new plane
        v12 = P2 - P1 # vector from P1 to P2
        v13 = P3 - P1 # vector from P1 to P3
        z_temp = np.cross(v12, v13) # vector orthogonal to v12 and v13

        U = np.linalg.norm(v12) # Magnitude of v12 / x pos of P2 on new plane

        x_hat = v12 / U # unit vector from P1 to P2
        z_hat = z_temp / np.linalg.norm(z_temp) # unit vector orthogonal to plane
        y_hat = np.cross(z_hat, x_hat) # unit vector on plane and orthogonal to x_hat

        Vx = np.dot(v13, x_hat) # x pos of P3 on new plane
        Vy = np.dot(v13, y_hat) # y pos of P3 on new plane

        x = (r1**2 - r2**2 + U**2) / (2*U) # x pos of P4 on new plane
        y = (r1**2 - r3**2 +Vx**2 +Vy**2 - 2*Vx*x) / (2*Vy) # y pos of P4 on new plane
        z1 = np.sqrt(max(0,r1**2 - x**2 - y**2)) # offset of P4 from plane
        z2 = -z1

        # 2 possible answers
        P4_1 = P1 + x*x_hat + y*y_hat + z1*z_hat
        P4_2 = P1 + x*x_hat + y*y_hat + z2*z_hat

        # Choose based on which one is closer to where it started
        if (np.linalg.norm(P4_1-P4_last) < np.linalg.norm(P4_2-P4_last)):
            P4 = P4_1
        else:
            P4 = P4_2

        return P4
 
    def bump_step(self, spring_length):
        """
        Use trilateration to solve for new points after spring length is changed.
        Input: current suspension points and spring length
        Output: new suspension points
        =======================================================================
        Fixed:
            SP_in: spring chassis attachment
            R1: rocker axis 1
            R2: rocker axis 2
            TR_in: tie rod inboard attachment
            UCF: upper front
            UCR: upper rear
            LCF: lower front
            LCR: lower rear
        Moving:
            SP_out: spring rocker attachment
            PR_in: push rod rocker attachment
            PR_out: push rod knuckle attachment
            TR_out: tie rod upright attachment
            WC: wheel center
            SA: spindle axis
            UB: upper ball joint
            LB: lower ball joint
            WB: wheel base/contact patch
        """
        
        # R1, R2, SP_in -> SP_out
        self.SP_out = self.trilateration(self.R1, self.R2, self.SP_in, self.lR1_SP_out, self.lR2_SP_out, spring_length, self.SP_out)
        # R1, R2, SP_out -> PR_in
        self.PR_in = self.trilateration(self.R1, self.R2, self.SP_out, self.lR1_PR_in, self.lR2_PR_in, self.lSP_out_PR_in, self.PR_in)
        # PR_in, LCF, LCR -> PR_out
        self.PR_out = self.trilateration(self.PR_in, self.LCF, self.LCR, self.lPR_in_PR_out, self.lLCF_PR_out, self.lLCR_PR_out, self.PR_out)
        # PR_out, LCF, LCR -> LB
        self.LB = self.trilateration(self.PR_out, self.LCF, self.LCR, self.lPR_out_LB, self.lLCF_LB, self.lLCR_LB, self.LB)
        # LB, UCF, UCR -> UB
        self.UB = self.trilateration(self.LB, self.UCF, self.UCR, self.lLB_UB, self.lUCF_UB, self.lUCR_UB, self.UB)
        # TR_in, UB, LB -> TR_out
        self.TR_out = self.trilateration(self.TR_in, self.UB, self.LB, self.lTR_in_TR_out, self.lUB_TR_out, self.lLB_TR_out, self.TR_out)
        # TR_out, UB, LB -> SA
        self.SA = self.trilateration(self.TR_out, self.UB, self.LB, self.lTR_out_SA, self.lUB_SA, self.lLB_SA, self.SA)
        # TR_out, UB, LB -> WC
        self.WC = self.trilateration(self.TR_out, self.UB, self.LB, self.lTR_out_WC, self.lUB_WC, self.lLB_WC, self.WC)
        # UB, LB, WC -> WB
        self.WB = self.trilateration(self.UB, self.LB, self.WC, self.lUB_WB, self.lLB_WB, self.lWC_WB, self.WB)

    def find_ABCD(self, UB, LB, UCF, UCR, LCF, LCR, WC_x):
        """
        Find the projections of right wheel upper and lower outboard and inboard connections used for finding the instant center.
        Input: All units are in mm
            UB:  upper ball joint (x,y,z)
            LB:  lower ball joint (x,y,z)
            UCF: upper front (x,y,z)
            UCR: upper rear (x,y,z)
            LCF: lower front (x,y,z)
            LCR: lower rear (x,y,z)
            WC_x: x position of YZ plane for projection
        ==========================================================================================================
        Output: All units are in mm
            A:  upper ball joint projected into YZ plane passing through wheel center (WB_x,y,z)
            B:  interpolation of upper inboard points on YZ plane passing through wheel center (WB_x,y,z)
            C:  lower ball joint projected into YZ plane passing through wheel center (WB_x,y,z)
            D:  interpolation of lower inboard points on YZ plane passing through wheel center (WB_x,y,z)
        """

        A = np.array([WC_x, UB[1], UB[2]])

        upper_ratio = (WC_x - UCF[0]) / (UCR[0] - UCF[0]) # x ratio between WC and UCR taken from UCF
        B = np.zeros(3)
        B[0] = WC_x
        B[1] = UCF[1] + upper_ratio*(UCR[1] - UCF[1])
        B[2] = UCF[2] + upper_ratio*(UCR[2] - UCF[2])

        C = np.array([WC_x, LB[1], LB[2]])

        lower_ratio = (WC_x - LCF[0]) / (LCR[0] - LCF[0]) # x ratio between WC and LCR taken from LCF
        D = np.zeros(3)
        D[0] = WC_x
        D[1] = LCF[1] + lower_ratio*(LCR[1] - LCF[1])
        D[2] = LCF[2] + lower_ratio*(LCR[2] - LCF[2])

        return A, B, C, D

    def susParam(self, FG=None, WC=None, SA=None, UB=None, LB=None, UCF=None, UCR=None, LCF=None, LCR=None, WB=None):
        """
        Calculate the suspension parameters for the right wheel.
        Input: All units are in mm
            FG: initial wheel center (x,y,z)
            WC: wheel center (x,y,z)
            SA: spindle axis (x,y,z)
            UB: upper ball joint (x,y,z)
            LB: lower ball joint (x,y,z)
            UCF: upper front inboard (x,y,z)
            UCR: upper rear inboard (x,y,z)
            LCF: lower front inboard (x,y,z)
            LCR: lower rear inboard (x,y,z)
            WB: wheel base/contact patch (x,y,z)
        =====================================================================================================
        Output: Length units are in mm, angle units are in degrees
            BM:    bump movement (+z movement)
            HTC:   half-track change (+y movement)
            WR:    wheel recession (+x movement)
            gamma: camber angle (positive is leaning outside)
            delta: steering/toe angle (positive is pointing inside)
            phi:   castor angle (positive is pointing to the front)
            TR:    mechanical trail (positive is to the front of the wheel)
            theta: kingpin inclination/kpi (positive if top of steering axis leans inward)
            GO:    ground-level offset/scrub (positive is to the inside of the wheel)
            RZ:    roll center height
        """

        if FG is None:
            FG = self.FG
        if WC is None:
            WC = self.WC
        if SA is None:
            SA = self.SA
        if UB is None:
            UB = self.UB
        if LB is None:
            LB = self.LB
        if UCF is None:
            UCF = self.UCF
        if UCR is None:
            UCR = self.UCR
        if LCF is None:
            LCF = self.LCF
        if LCR is None:
            LCR = self.LCR
        if WB is None:
            WB = self.WB    

        BM = WC[2] - FG[2]
        HTC = WC[1] - FG[1]
        WR = WC[0] - FG[0]

        gamma = (180 / np.pi) * np.arctan((WC[2]-SA[2])/(SA[1]-WC[1]))
        delta = (180 / np.pi) * np.arctan((WC[0]-SA[0])/(SA[1]-WC[1]))

        phi = (180 / np.pi) * np.arctan((UB[0]-LB[0])/(UB[2]-LB[2]))
        TR = WB[0]-LB[0] + (LB[2]-WB[2])*(UB[0]-LB[0])/(UB[2]-LB[2])

        theta = (180 / np.pi) * np.arctan((LB[1]-UB[1])/(UB[2]-LB[2]))
        GO = WB[1]-LB[1] - (LB[2]-WB[2])*(LB[1]-UB[1])/(UB[2]-LB[2])

        A, B, C, D = self.find_ABCD(UB, LB, UCF, UCR, LCF, LCR, WC[0])
        GR1 = (B[2]-A[2])/(B[1]-A[1])
        GR2 = (D[2]-C[2])/(D[1]-C[1])
        IC = np.zeros(3)
        IC[0] = WC[0]
        IC[1] = (GR1*A[1] -GR2*C[1] + C[2] - A[2]) / (GR1-GR2)
        IC[2] = A[2] + GR1*(IC[1]-A[1])

        GR3 = (IC[2]-WB[2]) / (IC[1]-WB[1])
        RCZ = WB[2] - GR3*WB[1]

        return BM, HTC, WR, gamma, delta, phi, TR, theta, GO, RCZ

    def wheel_rate(self, ks, spring_length, bump_change):
        """
        Returns suspension wheel rate.
        Input: spring stiffness (N/m), spring displacement (mm), wheel vertical displacement (mm)
        =========================================================================================
        Output: Spring force (N) and Wheel rate (N/m)
        """

        delta_spring = spring_length - self.lSP_in_SP_out
        Fs = -ks * (delta_spring*1000)
        kw = Fs / (bump_change*1000)

        return Fs, kw
    
    def plot(self):
        # Plot parameters vs bump
        length_units = "mm"
        angle_units = "degree"
        force_units = "N"
        X = self.BM_list

        # Initialise the subplot function using number of rows and columns 
        figure, axis = plt.subplots(2, 2) 
        
        # Camber
        axis[0, 0].plot(X,self.gamma_list) 
        axis[0, 0].set_title("Camber vs Bump")
        axis[0, 0].set_ylabel(angle_units)
        
        # Toe
        axis[0, 1].plot(X, self.delta_list) 
        axis[0, 1].set_title("Toe vs Bump") 
        
        # Castor
        axis[1, 0].plot(X, self.phi_list)
        axis[1, 0].set_title("Castor vs Bump")
        axis[1, 0].set_ylabel(angle_units)
        axis[1, 0].set_xlabel(length_units)
        
        # KPI 
        axis[1, 1].plot(X, self.theta_list) 
        axis[1, 1].set_title("KPI vs Bump")
        axis[1, 1].set_xlabel(length_units)
        
        # Initialise the subplot function using number of rows and columns 
        figure2, axis2 = plt.subplots(2, 3) 

        # HTC
        axis2[0, 0].plot(X,self.HTC_list)
        axis2[0, 0].set_title("Half Track Change vs Bump")
        axis2[0, 0].set_ylabel(length_units)
        
        # WR
        axis2[0, 1].plot(X, self.WR_list) 
        axis2[0, 1].set_title("Wheel Recession vs Bump")

        # Spring length
        axis2[0, 2].plot(X, self.spring-self.lSP_in_SP_out) 
        axis2[0, 2].set_title("Spring Displacement vs Bump")
        
        # Mechanical trail
        axis2[1, 0].plot(X, self.TR_list)
        axis2[1, 0].set_title("Mechanical Trail vs Bump")
        axis2[1, 0].set_ylabel(length_units)
        axis2[1, 0].set_xlabel(length_units)
        
        # Scrub
        axis2[1, 1].plot(X, self.GO_list) 
        axis2[1, 1].set_title("Scrub vs Bump")
        axis2[1, 1].set_xlabel(length_units)

        # Roll Center Height
        axis2[1, 2].plot(X, self.RZ_list) 
        axis2[1, 2].set_title("Roll Center Height vs Bump")
        axis2[1, 2].set_xlabel(length_units)

        # Combine all the operations and display 
        plt.show() 

    def solve(self):
        # Solve for the entire simulation
        i = 0
        for springLength in self.spring:
            self.bump_step(springLength)
            self.BM_list[i], self.HTC_list[i], self.WR_list[i], self.gamma_list[i], self.delta_list[i], self.phi_list[i], self.TR_list[i], self.theta_list[i], self.GO_list[i], self.RZ_list[i] = self.susParam()
            i = i+1

        if self.isPlot:
            self.plot()

if __name__ == "__main__":

    """
    test = kinSolve([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], np.array([0, 0, 0]), True)

    # Test plot
    test.plot()

    # Test trilateration
    P1 = np.array([2, 2, 0])
    P2 = np.array([3, 3, 0])
    P3 = np.array([1, 4, 0])
    P4 = test.trilateration(P1, P2, P3, 1, 1, np.sqrt(2), np.array([1.9, 3.1, 0]))
    print(P4) # should be [2, 3, 0]
    """