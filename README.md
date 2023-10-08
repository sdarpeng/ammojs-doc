# ammojs-doc
A non-official ammo.js documentation
forked from [zhaogong](http://www.dwenzhao.cn/profession/netbuild/ammoegine.html)

the items which signed * means it was updated by me and needs to be checked again.

# 一、常用的类：

## 1. btVector3类：三维向量类
该类使用频率很高，由3个浮点数类型的x、y、z变量组成，可以表示速度、点、力等向量。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btVector3()|创建一个三维分量初始值都为零的三维向量对象|
|btVector3(x,y,z)|创建三维向量对象，三个坐标值分别为x、y、z|
|2）方法：||
|方法|含义|
|setX(x)|设置向量的x坐标值|
|setY(y)|设置向量的y坐标值|
|setY(z)|设置向量的z坐标值|
|setValue(x,y,z)|设置向量的坐标|
|normalize()|获取向量归一化后的单位向量|
|dot(btVector3 v)|获取与向量v的点积|
|op_mul(btVector3 v)|获取与向量v的叉积|
|op_add(btVector3 v)|获取与向量v的和|
|op_sub(btVector3 v)|获取与向量v的差|
## 2. btTransform类：变换类
该类由位置和方向组合而成，用来表示刚体的变换，如平移、旋转等。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btTransform()|无参构造函数|
|btTransform(btQuaternion q, btVector3 v)|变换的构造函数，q表示变换旋转信息的四元数，v表示变换平移信息的向量|
|2）方法：||
|方法|含义|
|setIdentity()|将当前变换对象设置为初始状态，即将旋转变换矩阵归一化，平移向量3个维度的分量归零|
|setOrigin(btVector3 origin)|设置平移变换的向量，origin为平移变换的3x3矩阵|
|setRotation(btQuaternion rotation)|设置当前变换对象的旋转变换数据，rotation表示存储旋转数据的四元数对象|
|getOrigin()|换取变换的原点|
|getRotation()|换取表示旋转信息的四元数|
|getBasic()|换取表示变换信息的3x3矩阵|
|setFromOpenGLMatrix(m)|设置变换的矩阵，m为旋转平移缩放向量合成的4x4变换矩阵首地址|
|setEulerZYX( float, float, float )|设置欧拉变换|
|btQuaternion类表示的四元数，用于对三维向量进行变换。||
## 3. btRigidBody类：刚体类
该类用于存储刚体的一些属性信息，包括线速度、角速度、摩擦系数等，其中封装了多种方法，用于设置和获取相关属性信息。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btRigidBody(btRigidBodyConstructionInfo constructionInfo)|创建一个刚体对象，constructionInfo为刚体信息对象|
|2）方法：||
|方法|含义|
|getCenterOfMassTransform()|获取重心的变换，返回值为获取的四元数|
|setCenterOfMassTransform(btTransform xform)|设置刚体变换，参数xform表示需要变换的对象|
|setDamping(lin_damping, ang_damping)|设置现行阻尼系数和角阻尼系数|
|getLinearVelocity()|获取线速度，返回值为获取的线速度向量|
|getAngularVelocity()|获取加速度，返回值为获取的角速度向量|
|setAngularFactor(btVector3 angularFactort)|获取角度因子，angularFactort为要设置的角度因子|
|getMotionState()|获取刚体的形状，返回值为获取的形状指针|
|applyCentralForce(btVector3 force)|应用中心力，force为提供的力向量|
|applyTorch(btVector3 torquel)|应用扭矩，torque为要应用的刚体扭矩|
|applyForce(btVector3 force, btVector3 rel_pos)|应用力，force为要应用的力，rel_pos为施加力的位置|
|applyCentralImpulse(btVector3 impulse)|应用中心冲量，impulse为要应用的冲量|
|applyTorqueImpulse(btVector3 torque)|应用扭矩冲量，torque为要应用的冲量|
|applyImpulse ( btVector3 impulse, btVector3 rel_pos)|应用冲量，impulse为要应用的冲量，rel_pos为要施加冲量的位置坐标|
|setLinearVelocity( btVector3 )|设置线性速度|
|setAngularVelocity( btVector3 )|设置角速度|
|setContactProcessingThreshold( float)|设置接触（碰撞）检测阈值，一般会设的很大，例如100,000,000|
|*setDeactivationTime( float )|设置不活跃时长|
|*setSleepingThresholds( float, float )|设置睡眠阈值|
|*setFriction( float factor)| 设置摩擦力|
|*setRollingFriction( float factor)|设置滚动摩擦力|
|*setActivationState( stateIndex default: 4)||
|*setCollisionFlags( flags defualt:)|CF_STATIC_OBJECT = 1,  CF_KINEMATIC_OBJECT = 2,  CF_NO_CONTACT_RESPONSE = 4,  CF_CUSTOM_MATERIAL_CALLBACK = 8,  CF_CHARACTER_OBJECT = 16,  CF_DISABLE_VISUALIZE_OBJECT = 32,  CF_DISABLE_SPU_COLLISION_PROCESSING = 64,  CF_HAS_CONTACT_STIFFNESS_DAMPING = 128,  CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR = 256,  CF_HAS_FRICTION_ANCHOR = 512,  CF_HAS_COLLISION_SOUND_TRIGGER = 1024|
## 4. btDynamicsWorld类：物理世界类
该类有两个重要的子类，离散物理世界类btDiscreteDynamicsWorld和用于测试的类btSimpleDynamicsWorld类。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btDynamicsWorld ( btbtDispatcher dispatcher, btBroadphaseInterface broadphase, btCollisionConfiguration conf)|物理世界类构造器，dispatcher为碰撞检测算法分配器引用，conf为碰撞检测配置信息|
|2）方法：||
|方法|含义|
|stepSimulation(timeStep)|进行世界物理模拟，timeStep为时间步进|
|addConstraint(btTypedConstraint constraint)|在物理世界中添加约束，constraint为约束引用|
|removeConstraint(btTypedConstraint constraint)|在物理世界删除约束，constraint为约束引用|
|setGravity(gravity)|设置物理世界的重力，gravity为重力向量|
|addRidgidBody(btRidgidBody body)|在物理世界添加刚体，body为要添加的刚体|
|removeRidgidBody(btRidgidBody body)|删除物理世界的刚体|
|getNumConstraint()|获取物理世界的约束总数|
|getConstraint(index)|获取物理世界中的指定约束，index为约束索引|
|getNumCollisionObjects()|获取物理世界中碰撞物体的数量|
|getCollisionObjectArray()|获取物理世界中碰撞物体的数组|
|contactTest ( btCollisionObject colObj, ContactResultCallback resultCallback)|进行接触检测，colObj为指向碰撞物体类的引用，resultCallback为接触回调类的对象|
|getDispatcher()|获取物理世界的事件|
## 5. btDiscreteDynamicsWorld类：离散物理世界类
实际开发中常使用该类来创建物理世界对象，创建时要使用构造器，需要给出碰撞检测算法分配器、碰撞检测粗测算法接口和碰撞检测配置接口。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btDiscreteDynamicsWorld ( btbtDispatcher dispatcher, btBroadphaseInterface pairCache,btConstraintSolver constraintSolver, btCollisionConfiguration conf)|离散物理世界类构造器，dispatcher为碰撞检测算法分配器引用，pairCache为碰撞粗测算法接口，constraintSolver为约束解决器引用，conf为碰撞检测配置信息|
|2）方法：||
|方法|含义|
|btCollisionWorld getCollisionWorld()|获取当前物理世界的引用|
## 6. btSoftRigidDynamicsWorld类：支持模拟软体的物理世界
可支持模拟软体，继承了btDiscreteDynamicsWorld类。所谓软体，不具有固定形状，可像软布一样改变本身形状的物体。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btSoftRigidDynamicsWorld ( btbtDispatcher dispatcher, btBroadphaseInterface pairCache,btConstraintSolver constraintSolver, btCollisionConfiguration conf, btSoftBodySolver softBodySolver)|离散物理世界类构造器，dispatcher为碰撞检测算法分配器引用，pairCache为碰撞粗测算法接口，constraintSolver为约束解决器引用，conf为碰撞检测配置信息|
|2）方法：||
|方法|含义|
|addSoftBody(btSoftBody body)|向物理世界添加物体，body为指向软体的引用|
|removeSoftBody(btSoftBody body)|从物理世界删除指定软体|
## 7. btCollisionShape类：碰撞形状类
该类封装了一些判断碰撞形状类型的方法，所有碰撞形状都直接或间接继承自此类。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btCollisionShape ( )|碰撞形状构造器|
|2）方法：||
|方法|含义|
|setLocalScaling(btVector3 scaling)|设置缩放比|
|calculateLocalInertia(mass, btVector3 inertia)|计算惯性，mass为质量，inertia为惯性|
|setMargin(margin)|设置碰撞形状边缘数|
|getMargin()|获取碰撞形状边缘数|
## 8. btBoxShape类：长方体盒碰撞形状
该类可用于盒子、箱子等规则物体。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btBoxShape(btVector3 boxHalfExtents)|构造器，boxHalfExtents表示立方体盒子的半区域|
|2）方法：||
|方法|含义|
|setMargin(margin)|设置碰撞形状边缘数|
|getMargin()|获取碰撞形状边缘数|
## 9. btStaticPlaneShape类：静态平面形状
该类表示静态的平面，如地面、屋顶等，创建时需要给出法向量。
|  ||
|:--|:--|
|1）构造器为：||
|构造器|含义|
|btStaticPlaneShape(btVector3 planeNormal, float planeConstant)|静态平面构造器，参数planeNormal为平面法向量，planeConstant为平面上任意一点|
|2）方法：||
|方法|含义|
|getPlaneNormal()|获取平面形状的法向量|
## 10. btSphereShape类：球体形状
该类表示一个球体。
|  ||
|:--|:--|
|1）构造器为：||
|构造器|含义|
|btSphereShape(radius)|球体碰撞形状构造器，radius为球半径|
|2）方法：||
|方法|含义|
|setMargin(margin)|设置碰撞形状边缘数|
|getMargin()|获取碰撞形状边缘数|
|getRadius( )|获取球的半径|
## 11. btCylinderShape类：圆柱形状
该类表示一个圆柱形状，如杆、金币、石柱等都可以采用此类，但碰撞计算量较大，不如胶囊。
|  ||
|:--|:--|
|1）构造器为：||
|构造器|含义|
|btCylinderShape(btVector3 halfExtents)|圆柱对象构造器，halfExtents为圆柱的半区域，三维分量，第1和3维表示圆柱的长短半径，第2维是长度|
|2）方法：||
|方法|含义|
|getRadius( )|获取圆柱的半径|
## 12. btCapsuleShape类：胶囊形状
该类表示一个胶囊形状，碰撞计算量比圆柱小，旗杆、铅笔等一般使用该类。
|  ||
|:--|:--|
|1）构造器为：||
|构造器|含义|
|btCapsuleShape(float radius, float height)|胶囊碰撞形状对象构造器，参数radius为两端球面的半径，height为中间圆柱的长度|
|2）方法：||
|方法|含义|
|getRadius( )|获取胶囊截面的半径|
|getHalfHeight( )|获取中间圆柱部分长度值的一半|
## 13. btConeShape类：圆锥形状类
该类表示圆锥形状。
|  ||
|:--|:--|
|1）构造器为：||
|构造器|含义|
|btConeShape(float radius, float height)|圆锥碰撞形状对象构造器，参数radius为圆锥的半径，height为圆锥的高度|
|2）方法：||
|方法|含义|
|getRadius( )|获取圆锥的半径|
## 14. btCompoundShape类：复合形状
该类表示一个复合形状，可以通过创建多个单一形状组合成一个复合形状对象。
|  ||
|:--|:--|
|1）构造器为：||
|构造器|含义|
|btCompoundShape()|复合形状构造器|
|2）方法：||
|方法|含义|
|addChildShape ( btTransform localTransform, btCollisionShape shape)|向组合形状中添加子形状，localTransform为子形状的变换，shape为添加的子形状|
|removeChildShape( childShapeindex)|从组合形状中删除指定的子形状， childShapeindex为子形状索引|
|getNumChildShapes()|获取当前组合形状中子形状的数量|
|getChildShape(index)|获取组合形状中指定索引编号的子形状，index为子形状索引|
## 15. btRaycastVehicle类：交通工具类
交通工具类是模拟现实世界中的交通工具，有刚体车身、四个轮子，支持前轮驱动和后轮驱动，支持车轮转向等，提供了添加和更新车轮的方法，设置车轮刹车的方法。
|  ||
|:--|:--|
|方法|含义|
|updateAction ( btCollisionWorld collisionWorld, btScalar step)|更新交通工具，collisionWorld为物理世界的引用，step为步长|
|btTransform getChassisWorldTransform()|获取交通工具的变换对象|
|updateVehicle(btScalar step)|更新交通工具，step为更新的步长|
|resetSuspension()|重置悬挂系统的参数|
|btScalar getSteeringValue (wheelindex)|获取操纵车轮的系数，wheelindex表示车轮索引值|
|setSteeringValue(steering, wheelindex)|设置操纵车轮系数的值，steering为要设置的值|
|applyEngineForce(btScalar force, int  wheelindex)|车轮上应用力，force为力的大小，wheelindex表示车轮索引值|
|updateWheelTransform(wheelindex)|更新车轮的变换对象，wheelindex表示车轮索引值|
|btWheelInfo addWheel ( btVector3 connectionPointCS0, btVector3 wheelDirectionCS0, btVector3 wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRsdius, btVehicleTuning tuning,boolean isFrontWheel)|给交通工具添加车轮，connectionPointCS0为车轮连接点，wheelDirectionCS0为车轮方向，wheelAxleCS为车轮的轴向量，suspensionRestLength为车轮悬挂系统在松弛态下的长度，wheelRsdius为车轮半径，tuning为协调器，isFrontWheel为是否添加驱动力|
|getNumWheels()|获取交通工具上的车轮总数|
|btWheelInfo getWheelInfo(index)|获取交通工具上的车轮，index为车轮索引|
|setBrake(btScalar brake, index)|设置刹车系数，brake为要设置的刹车系数|
|updateSuspension (btScalar deltaTime)|更新悬挂系统，deltaTime为更新步长|
|updateFriction(btScalar timeStep)|更新摩擦，timeStep为更新步长|
|btRigidBody getRigidBody()|获取交通工具刚体|
|btVector3 getForwardVector()|获取交通工具的前进向量|
|btScalar getCurrentSpeedKmHour()|获取交通工具的当前速度|
|setCoordinateSystem(rightIndex, upIndex, forwardIndex)|设置坐标系统，rightIndex为右方向的索引，upIndex为上方向索引，forwardIndex为前进方向索引|
|getUserConstrainType ()|获取关节类型|
|setUserConstrainType(userConstraintType)|设置关节类型|
|setUserConstraintId(uid)|设置关节id|
|getUserConstraintId()|获取关节id|
## 16. btSoftBodyHelpers类：软体
软体是不同于固定形状的刚体，如绳索，可以实现拉伸、弯曲等不同姿态，如软布可以呈现上下波动。创建软体时必须使用软体帮助类，该类提供了创建软体的方法：
|  ||
|:--|:--|
|方法|含义|
|CreateRope ( btSoftBodyWorldInfo worldInfo, btVector3 from, btVector3 to, res, fixeds)|创建绳索软体的方法，worldInfo为软体世界信息，from为绳索起点位置，to为绳索终点位置，res为恢复系数，fixeds为坚硬系数|
|CreatePatch(btSoftBodyWorldInfo worldInfo, btVector3 corner00, btVector3 corner10, btVector3 corner01, btVector3 corner11, resx, resy, fixeds, boolean gendiags)|创建软布的方法，worldInfo为软体世界信息，corner00、corner10、corner01、corner11为软布四个角的坐标，resx为顶点列数，resy为顶点行数，gendiags为软布四角是否固定，true表示固定|
|CreateEllipsoid(btSoftBodyWorldInfo worldInfo, btVector3 center, btVector3 radius, res)|创建球软体的方法，worldInfo为软体世界信息，center为中心点坐标，radius为半径，res为恢复系数|
|CreateFromTriMesh(btSoftBody worldInfo, vertives, triangles, ntriangles, boolean randomizeConstraints)|创建三角形网络软体的方法，worldInfo为软体世界信息，vertices为顶点数组坐标，triangles为顶点索引数组，ntriangles为三角形总数|
## 17. btSoftBody类：软体
|  ||
|:--|:--|
|方法|含义|
|get_m_cfg()||
|set_viterations (int)||
|set_piterations (int)||
|set_kDF ( float )||
|set_kDP ( float )||
|set_kPR ( float )|设置压力|
|set_m_kLST ( float )| | 
|set_m_kAST ( float )| | 
|setTotalMass ( float )||
# 二、关节：
关节是两个物体之间的约束，关节的父类为btTypedConstraint类，其他关节都继承自该类，其封装了具体关节的共用方法。
|  ||
|:--|:--|
|1）构造器为：||
|构造器|含义|
|btTypedConstraint()|关节构造器|
|2）方法：||
|方法|含义|
|getBreakingImpulseThreshold()|获取毁坏关节的最大冲量|
|setBreakingImpulseThreshold( threshold)|设置毁坏关节的最大冲量， threshold为要设置的冲量值|
|*get_m_setting()|获取关节的设定信息|
|*set_m_impulseClamp( float )|设置关节的冲凉钳制参数|
|*set_m_tau( float)|设置那啥|
|*getConstraintType()|获取关节类型|

|关节主要有铰链关节、滑动关节、六自由度关节、点对点关节等。||
## 1. 铰链关节btHingeConstraint：
铰链是仅有一个旋转自由度的关节，通过铰链的约束限制，相关刚体仅能绕铰链轴旋转。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btHingeConstraint ( btRigidBody rbA, btRigidBody rbB, btVector3 pivotInA, btVector3 pivotInB, btVector3 axisInA, btVector3 axisInB, boolean useReferenceFrameA)|铰链构造器，参数rbA和rbB为要添加约束的两个刚体，pivotInA和pivotInB分别为对应的中心点，axisInA和axisInB为两个刚体的轴向量，useReferenceFrameA为两个刚体之间的约束关系，正常对应还是交叉对应，默认false|
|btHingeConstraint(btRigidBody rbA, btVector3 pivotInA, btVector3 axisInA, boolean useReferenceFrameA)|铰链构造器，参数rbA为要添加约束的两个刚体，pivotInA为对应的中心点，axisInA为两个刚体的轴向量，useReferenceFrameA为两个刚体之间的约束关系|
|btHingeConstraint(btRigidBody rbA, btRigidBody rbB, btTransform rbAFrame, btTransform rbBFrame,  boolean useReferenceFrameA)|铰链构造器，rbAFrame为第1个刚体的变换对象，rbBFrameaxisInB为第2个刚体的变换对象，useReferenceFrameA为两个刚体之间的约束关系，正常对应还是交叉对应|
|btHingeConstraint(btRigidBody rbA,  btTransform rbAFrame,  boolean useReferenceFrameA)|铰链构造器，rbAFrame为刚体的变换对象，useReferenceFrameA表示rbA是否与rbAFrame对应，默认false|
|2）方法：||
|方法|含义|
|getHingeAngle()|获取铰链当前的旋转角度值|
|setLimit ( float low, float high)|设置铰链的转动范围，low为下限值，high为上限值|
|getLowerLimit()|获取转动角度的下限值|
|getUpperLimit()|获取转动角度的上限值|
|enableAngularMotor ( boolean enableMotor, float targetVelocity, float maxMotorImpulse)|启动马达，enableMotor为是否允许使用马达，targetVelocity为关节角速度，maxMotorImpulse为最大马达驱动力|
|setAngularOnly(boolean angularOnly)|设置是否只开启角转动|
|enableMotor(boolean enableMotor)|设置是否开启马达|
|setMaxMotorImpulse(maxMotorImpluse)|设置马达的最大冲量|
|马达用于模拟提供动力的部件。||
## 2. 滑动关节btSliderConstraint：
滑动关节是一种仅有平移和旋转自由度的关节，如螺丝和螺母。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btSliderConstraint(btRigidBody rbA, btRigidBody rbB, btTransform frameInA, btTransform frameInB, boolean useLinearReferenceFrameA)|滑动关节构造器，rbA和rbB为两个刚体，frameInA和frameInB分别为从约束位置到刚体质心位置的变换，useLinearReferenceFrameA表示两个刚体与两个约束之间的对应关系，为true时rbA对应frameInA，为false时交叉对应|
|btSliderConstraint(btRigidBody rbA, btTransform frameInA, boolean useLinearReferenceFrameA)|滑动关节构造器，rbA为刚体，frameInA为从约束位置到刚体质心位置的变换，useLinearReferenceFrameA表示刚体与约束之间的对应关系|
|2）方法：||
|方法|含义|
|setUpperLinLimit ( float upperLimit)|设置滑动关节滑动距离上限|
|setLowerLinLimit ( float lowerLimit)|设置滑动关节滑动距离下限|
|getUpperLinLimit()|获取滑动距离的上限值|
|getLowerLinLimit()|获取滑动距离的下限值|
|setUpperAngLimit ( float upperLimit)|设置滑动关节转动角度上限|
|setLowerAngLimit ( float lowerLimit)|设置滑动关节转动角度下限|
|getUpperAngLimit()|获取转动角度的上限值|
|getLowerAngLimit()|获取转动角度的下限值|
|setDampingDirLin(float dampingDirLin)|设置关节的滑动阻尼系数|
|setDampingDirAng(float dampingDirAng)|设置关节的转动阻尼系数|
|getDampingDirLin()|获取关节的滑动阻尼系数|
|getDampingDirAng()|获取关节的转动阻尼系数|
|setPoweredLinMotor ( boolean onOff)|设置是否启动滑动对应的马达|
|setMaxLinMotorForce (float maxLinMotorForce)|设置驱动滑动马达的最大力|
|setTargetLinMotorVelocity (float targetLinMotorVelocity)|设置驱动滑动马达的速度|
|setPoweredAngMotor ( boolean onOff)|设置是否启动转动对应的马达|
|setMaxAngMotorForce (float maxAngMotorForce)|设置驱动转动马达的最大力|
|setTargetAngMotorVelocity (float targetangMotorVelocity)|设置驱动转动马达的速度|
## 3. 齿轮关节btGearConstraint：
为了模拟现实世界中齿轮之间的转动效果。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btGearConstraint(btRigidBody rbA, btRigidBody rbB, btVector3 axisA, btVector3 axisA, float ratio)|齿轮关节构造器，rbA和rbB为两个刚体，axisA和axisB分别为两个刚体的轴向量，ratio为转动比例|
|2）方法：||
|方法|含义|
|setAxisA ( btVector3 axisA)|设置关联第1个刚体的轴向量|
|setAxisB ( btVector3 axisB)|设置关联第2个刚体的轴向量|
|setRatio ( ratio)|设置齿轮关节转动比例|
|gettAxisA()|获取关联第1个刚体的轴向量|
|gettAxisB()|获取关联第1个刚体的轴向量|
|getRatio()|获取齿轮关节的转动比例|
## 4. 点对点关节btPoint2PointConstraint：
点对点关节模拟了两个物体上某两个点呈现连接效果。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btPoint2PointConstraint(btRigidBody rbA, btRigidBody rbB, btVector3 pivotInA, btVector3 pivotInB)|点对点约束构造器，rbA和rbB为两个刚体，pivotInA和pivotInB为关节分别在两个刚体坐标系中的位置|
|btPoint2PointConstraint(btRigidBody rbA, btVector3 pivotInA)|点对点约束构造器，pivotInA为关节在此刚体坐标系中的位置|
|2）方法：||
|方法|含义|
|setPivotA ( btVector3 pivotA)|设置关节在第1个刚体坐标系中的位置|
|setPivotB ( btVector3 pivotB)|设置关节在第2个刚体坐标系中的位置|
|getPivotInA ()|获取关节在第1个刚体坐标系中的位置|
|getPivotInB ()|获取关节在第2个刚体坐标系中的位置|
## 5. 六自由度关节btGeneric6DofConstraint：
六自由度关节有6个不同的自由度，包括3个平移自由度和3个转动自由度，可以模拟动物关节及机械结构，如肘关节、颈关节、机械手臂等。通过锁死或限制此关节的某个或某几个自由度，可以模拟其他类型的关节。
|  ||
|:--|:--|
|1）构造器：||
|构造器|含义|
|btGeneric6DofConstraint(btRigidBody rbA, btRigidBody rbB, btTransform frameInA, btTransform frameInB, boolean useLinearReferenceFrameA)|6自由度关节构造器，rbA和rbB为两个刚体，frameInA和frameInB分别为从约束位置到刚体质心位置的变换，useLinearReferenceFrameA表示两个刚体与两个约束之间的对应关系，为true时rbA对应frameInA，为false时交叉对应|
|btGeneric6DofConstraint(btRigidBody rbA, btTransform frameInA, boolean useLinearReferenceFrameA)|6自由度关节构造器，rbA为刚体，frameInA为从约束位置到刚体质心位置的变换，useLinearReferenceFrameA表示两个刚体与两个约束之间的对应关系|
|2）方法：||
|方法|含义|
|setLinearUpperLimit (bt Vector3 linearUpper)|设置关节3个滑动自由度距离的上限|
|setLinearLowerLimit ( btVector3 linearLower)|设置关节3个滑动自由度距离的下限|
|seAngularUpperLimit ( btVector3 angularUpper)|设置关节3个转动自由度距离的上限|
|setAngularLowerLimit ( btVector3 angularLower)|设置关节3个转动自由度距离的下限|
|getAngular(int axis_index)|获取指定轴的旋转角度，基于欧拉角的计算方法|

# 三、回调
## 1、接触（碰撞）事件回调
|  ||
|:--|:--|
|*ConcreteContactResultCallback()|接触（碰撞）事件回调函数|
|1）构造器：||
|构造器|含义|
|*btManifoldPoint|多歧事件点|
|*btCollisionObjectWrapper||
|2）方法：||
|方法|含义|
|*addSingleResult()|调用回调函数一次并返回一组数据|
|*wrapPointer( objWrap, btCollisionObjectWrapper )||
|*getManifoldByIndexInternal( index )||
|*getBody0()|返回参与碰撞的第1个对象|
|*getBody1()|返回参与碰撞的第2个对象|
|*getDispatcher()|获取物理世界的事件|
|*getNumManifolds()|获取物理世界的多歧事件|
|*getNumContacts()|获取物理世界多歧事件中的接触（碰撞）事件数量|
|*getContactPoint()|获取上一条接触（碰撞）事件的碰撞点|
|*getDistance()|获取上一条碰撞点的距离|
## 2、射线检测回调  
|  ||
|:--|:--|
|*ClosestRayResultCallback()|射线检测回调函数|
|1）构造器：||
|构造器|含义|
|2）方法：||
|方法|含义|
|*hasHit()|结果为Bool，存在射线命中是，否|
|*get_m_collisionObject()|获取射线命中对象|
|*isStaticObject()|结果为Bool，判断对象是否为静止对象（质量为0）|
|*isKinematicObject()|结果为Bool，判断对象是否为（非冲量型）kinematic对象|

# 四、一些开发思路
## 1、鼠标拾取刚体效果的实现方案
常规的rigidBody或者softBody是只受到冲量影响，因此是不能被直接修改位置的。要实现鼠标拾取的效果，可按照如下的思路：  
1）、onmousedown()  
-在鼠标点击事件时，用raycast检测是否穿透了某个rigidBody/SoftBody；  
-在鼠标位置创建一个kinematicBody（可以直接修改位置，且接受碰撞）；  
-创建一个6dofconstraint或者p2pconstraint，用来连接新创建的kinematicBody和射线穿透的rigidBody/SoftBody；  
2）、onmouseup()  
-销毁刚刚创建的constraint；  
-销毁刚刚创建的kinematicBody；  
## 2、布料self-collision的实现  
毫无头绪。
