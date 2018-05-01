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
            print(current_frame.name + "\n")
            for child in current_frame.allChildren:
                print(child + "\n")
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


    gt.addFrame("shelf_layer2", "shelf_layer3", Transform(1,0,5))

    gt.addFrame("shelf_layer3", "shelf_layer4", Transform(1,0,5))

    gt.addFrame("shelf_layer4", "shelf_layer5", Transform(1,0,5))

    gt.addFrame("shelf_layer5", "shelf_layer6", Transform(1,0,5))
    #gt.addFrame("root", Frame("asd", Transform(1,0,0)))
    #gt.addFrame("root", Frame("dasa", Transform(1,1,0)))
    #gt.addFrame("asd", Frame("gsffsg", Transform(1,0,5)))
    gt.printTree(gt.root)
    #t = gt.getTransform("root","asd")
    t = gt.getTransform("root","shelf_layer1")
    print(str(t))
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