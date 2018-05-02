#!/usr/bin/env python
# TODO(Georg): add license
import rospy

class Transform:
    def __init__(self, x = 0 ,y = 0 ,z = 0):
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    def __str__(self):
        return ("x: " + str(self.x) + " y: " + str(self.y) + " z: " + str(self.z))

class Frame:
    def __init__(self, name, transform):
        self.name = name
        self.transform = transform
        self.children = []
        self.allChildren = []


class GroundTruth:

    def __init__(self):
        self.root = Frame("root", Transform(0,0,0))
        #self.root.allChildren.append("root")

    def addFrame(self, parent_frame_name, child_frame_name, transform):
        current_frame = self.root
        child_frame = Frame(child_frame_name, transform)

        if parent_frame_name == self.root.name:
            self.root.allChildren.append(child_frame_name)
            self.root.children.append(child_frame)
            return

        if parent_frame_name not in self.root.allChildren:
        	print("parent " + parent_frame_name + " does not exist")
        	return
            #print(current_frame.name + "\n")
            #for child in current_frame.allChildren:
            #    print(child + "\n")


        if child_frame_name in self.root.allChildren:
            print("Frame " + child_frame_name + " exists already")
            return

        while(current_frame.name != parent_frame_name):
            for next_frame in current_frame.children:
                if parent_frame_name in next_frame.allChildren:
                    current_frame.allChildren.append(child_frame_name)
                    current_frame = next_frame
                    continue
                if next_frame.name == parent_frame_name:
                    current_frame.allChildren.append(child_frame_name)
                    current_frame = next_frame
                    continue

        current_frame.allChildren.append(child_frame_name)
        current_frame.children.append(child_frame)


    def getRootTransform(self, frame_name):
        if(frame_name == self.root.name):
            return self.root.transform

        if frame_name not in self.root.allChildren:
            print("Frame " + frame_name +  " does not exist")
            return Transform()


        root_transform = self.root.transform
        t = Transform(root_transform.x, root_transform.y, root_transform.z)
        current_frame = self.root
        

        while(current_frame.name != frame_name):
            for next_frame in current_frame.children:
                if frame_name in next_frame.allChildren:
                    t = t * current_frame.transform
                    current_frame = next_frame
                    continue
                if next_frame.name == frame_name:
                    t = t * current_frame.transform
                    current_frame = next_frame
                    continue

        t = t * current_frame.transform

        return t




    def getTransform(self, frame1, frame2):
        t1 = self.getRootTransform(frame1)
        t2 = self.getRootTransform(frame2)
        return Transform(t2.x - t1.x, t2.y - t1.y, t2.z - t1.z)




    def printTree(self, frame, depth = 0):
        s = ""
        for i in range(0, depth):
            s = s + "     "
        print(s + "frame name: " + frame.name + " Transfrom: " + str(frame.transform))
        print("\n")
        #print("children size: " + str(len(frame.children)) + "\n")
        
        for child_frame in frame.children:
            #print ("child frame " + child_frame.name + " size " + str(len(child_frame.children)) + "\n")
            #time.sleep(1)
            self.printTree(child_frame, depth + 1)


def instantiate_ground_truth():
    layer_to_seperator_y = 0.01
    layer_to_seperator_z = 0.005

    layer_to_barcode_z = -0.035
    layer_to_barcode_y = -0.028


    gt = GroundTruth()
    gt.addFrame("root", "shelf1_root", Transform(1,0,0))
    gt.addFrame("shelf1_root", "shelf_layer1", Transform(0.02, -0.035, 0.14))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_1", Transform(0.03, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_2", Transform(0.1, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_3", Transform(0.24, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_4", Transform(0.382, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_5", Transform(0.511, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_6", Transform(0.64, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_7", Transform(0.766, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_8", Transform(0.867, 0.01, 0.005))
    gt.addFrame("shelf_layer1", "shelf_layer1_seperator_9", Transform(0.98, 0.01, 0.005))

    gt.addFrame("shelf_layer1", "027995", Transform(0.03, -0.028, -0.035))
    gt.addFrame("shelf_layer1", "544205", Transform(0.162, -0.028, -0.035))
    gt.addFrame("shelf_layer1", "384160", Transform(0.297, -0.028, -0.035))
    gt.addFrame("shelf_layer1", "457319", Transform(0.428, -0.028, -0.035))
    gt.addFrame("shelf_layer1", "534812", Transform(0.572, -0.028, -0.035))
    gt.addFrame("shelf_layer1", "402610", Transform(0.697, -0.028, -0.035))
    gt.addFrame("shelf_layer1", "433961", Transform(0.806, -0.028, -0.035))
    gt.addFrame("shelf_layer1", "507923", Transform(0.917, -0.028, -0.035))


    gt.addFrame("shelf_layer1", "shelf_layer2", Transform(0, 0.102, 0.4))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_1", Transform(0.03, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_2", Transform(0.155, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_3", Transform(0.218, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_4", Transform(0.28, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_5", Transform(0.361, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_6", Transform(0.561, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_7", Transform(0.609, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_8", Transform(0.749, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_9", Transform(0.858, 0.01, 0.005))
    gt.addFrame("shelf_layer2", "shelf_layer2_seperator_10", Transform(0.98, 0.01, 0.005))

    gt.addFrame("shelf_layer2", "500183", Transform(0.065, -0.028, -0.035))
    gt.addFrame("shelf_layer2", "046088", Transform(0.197, -0.028, -0.035))
    gt.addFrame("shelf_layer2", "262289", Transform(0.302, -0.028, -0.035))
    gt.addFrame("shelf_layer2", "010055", Transform(0.414, -0.028, -0.035))
    gt.addFrame("shelf_layer2", "015652", Transform(0.538, -0.028, -0.035))
    gt.addFrame("shelf_layer2", "516937", Transform(0.664, -0.028, -0.035))
    gt.addFrame("shelf_layer2", "125481", Transform(0.906, -0.028, -0.035))


    gt.addFrame("shelf_layer2", "shelf_layer3", Transform(0 ,0, 0.34))
    gt.addFrame("shelf_layer3", "shelf_layer3_seperator_1", Transform(0.030, 0.01, 0.005))
    gt.addFrame("shelf_layer3", "shelf_layer3_seperator_2", Transform(0.150, 0.01, 0.005))
    gt.addFrame("shelf_layer3", "shelf_layer3_seperator_3", Transform(0.364, 0.01, 0.005))
    gt.addFrame("shelf_layer3", "shelf_layer3_seperator_4", Transform(0.513, 0.01, 0.005))
    gt.addFrame("shelf_layer3", "shelf_layer3_seperator_5", Transform(0.771, 0.01, 0.005))
    gt.addFrame("shelf_layer3", "shelf_layer3_seperator_6", Transform(0.857, 0.01, 0.005))
    gt.addFrame("shelf_layer3", "shelf_layer3_seperator_7", Transform(0.999, 0.01, 0.005))

    gt.addFrame("shelf_layer3", "004728", Transform(0.056, -0.028, -0.035))
    gt.addFrame("shelf_layer3", "196068", Transform(0.240, -0.028, -0.035))
    gt.addFrame("shelf_layer3", "332384", Transform(0.426, -0.028, -0.035))
    gt.addFrame("shelf_layer3", "523129", Transform(0.584, -0.028, -0.035))
    gt.addFrame("shelf_layer3", "424929", Transform(0.772, -0.028, -0.035))
    gt.addFrame("shelf_layer3", "235542", Transform(0.902, -0.028, -0.035))

    gt.addFrame("shelf_layer3", "shelf_layer4", Transform(0 ,0, 0.298))
    gt.addFrame("shelf_layer4", "shelf_layer4_seperator_1", Transform(0.030, 0.01, 0.005))
    gt.addFrame("shelf_layer4", "shelf_layer4_seperator_2", Transform(0.181, 0.01, 0.005))
    gt.addFrame("shelf_layer4", "shelf_layer4_seperator_3", Transform(0.294, 0.01, 0.005))
    gt.addFrame("shelf_layer4", "shelf_layer4_seperator_4", Transform(0.433, 0.01, 0.005))
    gt.addFrame("shelf_layer4", "shelf_layer4_seperator_5", Transform(0.625, 0.01, 0.005))
    gt.addFrame("shelf_layer4", "shelf_layer4_seperator_6", Transform(0.824, 0.01, 0.005))
    gt.addFrame("shelf_layer4", "shelf_layer4_seperator_7", Transform(0.990, 0.01, 0.005))

    gt.addFrame("shelf_layer4", "114035", Transform(0.068, -0.028, -0.035))
    gt.addFrame("shelf_layer4", "176671", Transform(0.217, -0.028, -0.035))
    gt.addFrame("shelf_layer4", "422771", Transform(0.328, -0.028, -0.035))
    gt.addFrame("shelf_layer4", "331372", Transform(0.498, -0.028, -0.035))
    gt.addFrame("shelf_layer4", "377954", Transform(0.701, -0.028, -0.035))
    gt.addFrame("shelf_layer4", "227003", Transform(0.874, -0.028, -0.035))

    gt.addFrame("shelf_layer4", "shelf_layer5", Transform(0 ,0, 0.255))
    gt.addFrame("shelf_layer5", "shelf_layer5_seperator_1", Transform(0.030, 0.01, 0.005))
    gt.addFrame("shelf_layer5", "shelf_layer5_seperator_2", Transform(0.109, 0.01, 0.005))
    gt.addFrame("shelf_layer5", "shelf_layer5_seperator_3", Transform(0.308, 0.01, 0.005))
    gt.addFrame("shelf_layer5", "shelf_layer5_seperator_4", Transform(0.571, 0.01, 0.005))
    gt.addFrame("shelf_layer5", "shelf_layer5_seperator_5", Transform(0.774, 0.01, 0.005))
    gt.addFrame("shelf_layer5", "shelf_layer5_seperator_6", Transform(0.980, 0.01, 0.005))

    gt.addFrame("shelf_layer5", "200563", Transform(0.030, -0.028, -0.035))
    gt.addFrame("shelf_layer5", "447600", Transform(0.186, -0.028, -0.035))
    gt.addFrame("shelf_layer5", "501155", Transform(0.412, -0.028, -0.035))
    gt.addFrame("shelf_layer5", "501156", Transform(0.662, -0.028, -0.035))
    gt.addFrame("shelf_layer5", "534910", Transform(0.861, -0.028, -0.035))


    gt.addFrame("shelf_layer5", "shelf_layer6", Transform(0 ,0, 0.338))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_1", Transform(0.030, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_2", Transform(0.111, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_3", Transform(0.201, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_4", Transform(0.282, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_5", Transform(0.358, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_6", Transform(0.425, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_7", Transform(0.513, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_8", Transform(0.648, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_9", Transform(0.728, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_10", Transform(0.814, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_11", Transform(0.886, 0.01, 0.005))
    gt.addFrame("shelf_layer6", "shelf_layer6_seperator_12", Transform(0.982, 0.01, 0.005))

    gt.addFrame("shelf_layer6", "305553", Transform(0.044, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "503131", Transform(0.140, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "333192", Transform(0.219, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "333140", Transform(0.306, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "050081", Transform(0.371, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "454259", Transform(0.453, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "048180", Transform(0.565, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "046445", Transform(0.670, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "500124", Transform(0.755, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "333154", Transform(0.837, -0.028, -0.035))
    gt.addFrame("shelf_layer6", "023130", Transform(0.923, -0.028, -0.035))



    gt.addFrame("root", "shelf2_root", Transform(2,0,0))
    gt.addFrame("shelf2_root", "shelf2_layer1", Transform(0.02, -0.035, 0.14))

    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_1", Transform(-0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_2", Transform(0.051, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_3", Transform(0.086, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_4", Transform(0.129, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_5", Transform(0.181, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_6", Transform(0.389, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_7", Transform(0.589, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_8", Transform(0.785, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer1", "shelf2_layer1_seperator_9", Transform(0.991, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf2_layer1", "013462", Transform(0.030, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer1", "458823", Transform(0.113, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer1", "522273", Transform(0.268, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer1", "102696", Transform(0.466, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer1", "102694", Transform(0.676, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer1", "519151", Transform(0.848, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf2_layer1", "shelf2_layer2", Transform(0 , 0.102, 0.190))

    gt.addFrame("shelf2_layer2", "shelf2_layer2_seperator_1", Transform(0.010, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer2", "shelf2_layer2_seperator_2", Transform(0.194, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer2", "shelf2_layer2_seperator_3", Transform(0.381, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer2", "shelf2_layer2_seperator_4", Transform(0.575, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer2", "shelf2_layer2_seperator_5", Transform(0.775, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer2", "shelf2_layer2_seperator_6", Transform(0.975, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf2_layer2", "522990", Transform(0.106, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer2", "522991", Transform(0.274, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer2", "523009", Transform(0.446, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer2", "522985", Transform(0.653, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer2", "523010", Transform(0.841, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf2_layer2", "shelf2_layer3", Transform(0 , 0, 0.210))

    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_1", Transform(0.010, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_2", Transform(0.172, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_3", Transform(0.300, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_4", Transform(0.415, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_5", Transform(0.463, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_6", Transform(0.516, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_7", Transform(0.570, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_8", Transform(0.630, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_9", Transform(0.680, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_10", Transform(0.740, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_11", Transform(0.804, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_12", Transform(0.864, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_13", Transform(0.920, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf2_layer3", "shelf2_layer3_seperator_14", Transform(0.976, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf2_layer3", "522989", Transform(0.075, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "522988", Transform(0.235, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "294763", Transform(0.351, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "294760", Transform(0.425, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "449181", Transform(0.471, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "449182", Transform(0.530, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "507237", Transform(0.587, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "036942", Transform(0.641, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "036946", Transform(0.699, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "036945", Transform(0.763, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "036988", Transform(0.824, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer3", "190755", Transform(0.913, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf2_layer3", "shelf2_layer4", Transform(0 , 0.145, 0.495))

    gt.addFrame("shelf2_layer4", "463940", Transform(0.021, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "155777", Transform(0.086, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "539355", Transform(0.149, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "356346", Transform(0.198, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "513205", Transform(0.255, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "544140", Transform(0.302, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "325776", Transform(0.350, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "045907", Transform(0.393, layer_to_barcode_y, layer_to_barcode_z))   
    gt.addFrame("shelf2_layer4", "461819", Transform(0.445, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "264913", Transform(0.499, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "264912", Transform(0.541, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "107184", Transform(0.601, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "362767", Transform(0.680, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "477847", Transform(0.767, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "545202", Transform(0.843, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer4", "294993", Transform(0.924, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf2_layer4", "shelf2_layer5", Transform(0 , 0, 0.298))

    gt.addFrame("shelf2_layer5", "121582", Transform(0.020, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "039450", Transform(0.074, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "531041", Transform(0.128, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "531033", Transform(0.177, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "566325", Transform(0.230, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "096981", Transform(0.293, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "218409", Transform(0.350, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "499551", Transform(0.418, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "011850", Transform(0.471, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "011853", Transform(0.525, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "318407", Transform(0.586, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "049544", Transform(0.655, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "318497", Transform(0.705, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "431394", Transform(0.755, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "208112", Transform(0.809, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "472899", Transform(0.870, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer5", "472897", Transform(0.924, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf2_layer5", "shelf2_layer6", Transform(0 , 0, 0.254))

    gt.addFrame("shelf2_layer6", "349789", Transform(0.010, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "349799", Transform(0.090, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "349797", Transform(0.166, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "387457", Transform(0.256, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "515633", Transform(0.328, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "447775", Transform(0.404, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "535427", Transform(0.476, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "046255", Transform(0.575, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "479455", Transform(0.673, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "467637", Transform(0.755, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "317514", Transform(0.835, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf2_layer6", "317512", Transform(0.916, layer_to_barcode_y, layer_to_barcode_z))







    gt.addFrame("root", "shelf3_root", Transform(3,0,0))
    gt.addFrame("shelf3_root", "shelf3_layer1", Transform(0.02, -0.035, 0.14))

    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_1", Transform(0.030, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_2", Transform(0.090, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_3", Transform(0.285, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_4", Transform(0.380, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_5", Transform(0.470, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_6", Transform(0.559, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_7", Transform(0.645, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_8", Transform(0.735, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_9", Transform(0.821, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_10", Transform(0.907, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer1", "shelf3_layer1_seperator_11", Transform(0.995, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf3_layer1", "260725", Transform(0.075, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer1", "260707", Transform(0.270, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer1", "554301", Transform(0.402, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer1", "404491", Transform(0.502, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer1", "521873", Transform(0.634, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer1", "402589", Transform(0.745, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer1", "260667", Transform(0.889, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf3_layer1", "shelf3_layer2", Transform(0 , 0.102, 0.316))

    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_1", Transform(0.030, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_2", Transform(0.070, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_3", Transform(0.147, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_4", Transform(0.200, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_5", Transform(0.255, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_6", Transform(0.309, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_7", Transform(0.360, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_8", Transform(0.413, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_9", Transform(0.468, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_10", Transform(0.520, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_11", Transform(0.606, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_12", Transform(0.692, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_13", Transform(0.800, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_14", Transform(0.896, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer2", "shelf3_layer2_seperator_15", Transform(0.985, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf3_layer2", "520698", Transform(0.026, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "520696", Transform(0.096, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "520689", Transform(0.188, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "520688", Transform(0.295, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "423376", Transform(0.431, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "422068", Transform(0.550, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "511860", Transform(0.625, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "260700", Transform(0.736, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "260695", Transform(0.832, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer2", "399599", Transform(0.923, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf3_layer2", "shelf3_layer3", Transform(0 , 0, 0.296))

    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_1", Transform(0.030, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_2", Transform(0.072, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_3", Transform(0.140, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_4", Transform(0.194, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_5", Transform(0.261, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_6", Transform(0.386, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_7", Transform(0.443, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_8", Transform(0.508, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_9", Transform(0.596, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_10", Transform(0.732, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_11", Transform(0.800, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_12", Transform(0.868, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_13", Transform(0.923, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer3", "shelf3_layer3_seperator_14", Transform(0.987, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf3_layer3", "535675", Transform(0.020, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "519763", Transform(0.093, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "520209", Transform(0.151, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "520212", Transform(0.213, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "523828", Transform(0.399, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "523829", Transform(0.467, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "523823", Transform(0.542, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "523830", Transform(0.644, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "472787", Transform(0.750, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "293857", Transform(0.820, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "536979", Transform(0.886, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer3", "411522", Transform(0.951, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf3_layer3", "shelf3_layer4", Transform(0 , 0, 0.295))

    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_1", Transform(0.030, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_2", Transform(0.060, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_3", Transform(0.125, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_4", Transform(0.180, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_5", Transform(0.238, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_6", Transform(0.325, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_7", Transform(0.412, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_8", Transform(0.483, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_9", Transform(0.547, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_10", Transform(0.613, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_11", Transform(0.691, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_12", Transform(0.767, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_13", Transform(0.846, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_14", Transform(0.915, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer4", "shelf3_layer4_seperator_15", Transform(0.980, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf3_layer4", "460470", Transform(0.016, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "460201", Transform(0.080, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "536952", Transform(0.132, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "536943", Transform(0.192, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "536970", Transform(0.265, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "565329", Transform(0.351, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "565335", Transform(0.435, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "518875", Transform(0.503, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "536760", Transform(0.560, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "536803", Transform(0.645, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "518876", Transform(0.725, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "544375", Transform(0.794, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "544382", Transform(0.860, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer4", "553733", Transform(0.940, layer_to_barcode_y, layer_to_barcode_z))

    gt.addFrame("shelf3_layer4", "shelf3_layer5", Transform(0 , 0, 0.335))

    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_1", Transform(0.010, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_2", Transform(0.098, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_3", Transform(0.161, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_4", Transform(0.261, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_5", Transform(0.332, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_6", Transform(0.426, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_7", Transform(0.485, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_8", Transform(0.597, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_9", Transform(0.672, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_10", Transform(0.750, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_11", Transform(0.905, layer_to_seperator_y, layer_to_seperator_z))
    gt.addFrame("shelf3_layer5", "shelf3_layer5_seperator_12", Transform(0.983, layer_to_seperator_y, layer_to_seperator_z))

    gt.addFrame("shelf3_layer5", "544799", Transform(0.030, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "544760", Transform(0.112, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "544788", Transform(0.190, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "566217", Transform(0.280, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "544782", Transform(0.362, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "512018", Transform(0.445, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "512036", Transform(0.522, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "553735", Transform(0.652, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "553736", Transform(0.770, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "553726", Transform(0.847, layer_to_barcode_y, layer_to_barcode_z))
    gt.addFrame("shelf3_layer5", "553715", Transform(0.929, layer_to_barcode_y, layer_to_barcode_z))


    
    

    #gt.printTree(gt.root)
    t1 = gt.getTransform("shelf1_root","023130")
    t2 = gt.getTransform("305553","500124")
    t3 = gt.getTransform("shelf_layer5_seperator_3","shelf_layer3")
    #t = gt.getTransform("root","shelf_layer1")
    print(str(t1))
    print(str(t2))
    print(str(t3))
    # TODO(Kevin): please complete me; feel free to add as many helper functions as you need
    pass


def visualize(data):
    # TODO(Georg): please complete me
    pass


if __name__ == '__main__':
    try:
        visualize(instantiate_ground_truth())
    except rospy.ROSInterruptException:
        pass