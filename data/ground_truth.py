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
    gt.addFrame("shelf_layer2", "46088", Transform(0.197, -0.028, -0.035))
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