# 你必须知道的  
首先，普通的刚体只接受冲量（impulse）输入，不支持直接修改位置；  
其次，刚体在创建完成后，其碰撞类型（flag）不可以修改；  
因此，要想实现鼠标拾取刚体，你不可能直接通过修改刚体的position来实现。  

# 实现路径  
1、刚体中有一个特殊的flag类型：kinematic，这种类型的刚体不会受到场景中其他冲量的影响，反过来他可以施加冲量影响给其他的普通刚体，并且，你可以直接修改kinematic刚体的位置。  
2、刚体和约束可以创建和销毁。  
3、因此，你可以使用射线检测判断鼠标所在位置的射线是否击中了普通刚体，如果是，则在击中位置创建一个kinematic类型的刚体；  
4、将刚刚创建的kinematic刚体与你希望拾取的刚体通过约束（constraint）连接起来；  
5、然后，你需要将鼠标在屏幕坐标系的移动数据，通过射线Raycaster的击中点，转换成场景里的3D空间数据，但是，如果你的鼠标移动到空中，射线不再击中任何场景实体，你将无法继续获得击中点的位置，因为没有击中点；  
6、幸运的是，Threejs为我们准备了一个特殊的非Object3D类的工具，Math/Plane对象，这个Plane对象不可见，并且具有无限的广度，最关键的是，他能被Raycaster的射线击中！此处可以参考threejs官方examples/jsm/controls/dragControls的源代码；  
7、你需要在每一帧更新Plane的朝向，使其始终与相机的方向正交，你还需要在每一帧更新Plane的位置，使其与被拾取对象，在摄像机坐标系下拥有相同的深度；  
8、然后，你就可以在任何时候都获得鼠标光标在屏幕坐标系下移动的3D空间转换数据，而无论鼠标所指向的地方是否有Object3D对象；  
9、将获得的空间数据copy给Kinematic对象（需同时修改渲染用的ObjThree 和动力学用的ObjPhysics）；  
10、还有一个细节，你肯定不希望你在拖动刚体的时候，OrbitControl还跟着转个不停，因此，你需要给container注入pointerdown、pointermove、pointerup3个事件监听指令，当pointerdown的时候，你需要立即保存OrbitControl的状态(saveState)，然后使其无效（enabled =false），然后在鼠标取消拾取敢提的时候（onpointerup），再使OrbitControl重新生效。  
11、在pointerup的时候，销毁为了拾取刚体而创建的kinematic刚体和约束(constraint)。
