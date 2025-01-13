# 2025决策重构

目前已经基本完成了对上赛季决策代码信息结构的优化整理。

首先重申本次优化的主要动机：将决策代码分解为状态层，决策层和动作层三层，状态层和动作层将所有可以调用的接口打包整理后再提供给决策层，从而简化决策层的迭代和替换成本。决策层不论采用有限状态机，行为树，强化学习还是其他任何方案，都只需要对接少量统一的接口即可完成替换或升级。

下面将简要说明状态层和动作层规定的接口：

# 1. 状态层

涉及的功能包包括`information_widgets`，`iw_interfaces`和`sensing_unit`。

## 1.1 information\_widgets 信息组件

主要包含对信息组件`ChessboardHandle`和`Prism`类的定义。

### 1.1.1 ChessboardHandle类说明

ChessboardHandle类的成员构成逻辑上很简洁，由`faction`（阵营），`robot`（机器人），`terrain`（地形）和`architecture`（建筑）四种要素构成。

### ChessboardHandle（棋盘权柄）

| 属性                   | 类型                                                                               | 描述            |
| -------------------- | -------------------------------------------------------------------------------- | ------------- |
| 阵营（`faction`）        | `Faction`                                                                        | 棋盘持有者当前所属阵营   |
| 机械小队（`robots`）       | `std::shared_ptr<std::unordered_map`<br>`<std::string,std::shared_ptr<Robot>>>`        | 赛场上所有地面作战机器人  |
| 地形簇（`terrains`）      | `std::shared_ptr<std::unordered_map`<br>`<std::string,std::shared_ptr<Terrain>>>`      | 赛场上的涵洞，坡道等地形  |
| 建筑群（`architectures`） | `std::shared_ptr<std::unordered_map`<br>`<std::string,std::shared_ptr<Architecture>>>` | 赛场上的前哨站，基地等建筑 |

值得说明的是，ChessboardHandle类中的大多数成员全部都是以智能指针的形式出现的，这是为了：

1. 在同一节点内部，棋盘权柄可以大量分发而不产生额外的空间消耗；

2. 由于部分成员涉及到一定的继承关系，所以要避免在后续增加子类的时候出现对象切割的情况。

其中涉及到的四种类型又分别定义如下：

### Faction（阵营）

枚举类型，分为`UNKNOWN`，`RED`和`BLUE`三种，其设置和上赛季的整数设定兼容。

### Object（物体）

是机器人，地形和建筑的父类。

| 属性          | 类型            | 描述    |
| ----------- | ------------- | ----- |
| 标签（`label`） | `std::string` | 物体的标签 |

### Robot（机器人）

在上个赛季的基础上添加了部分属性。

| 属性              | 类型                                | 描述                               |
| --------------- | --------------------------------- | -------------------------------- |
| 阵营（`faction`）   | `faction`                         | 机器人所属阵营                          |
| id（`id`）        | `uint`                            | 机器人的数字id                         |
| 位姿（`pose`）      | `geometry_msgs::msg::PoseStamped` | 机器人的位姿，包括其三维坐标和四元数表示的偏转方向，带有时间戳。 |
| 生命值（`hp`）       | `uint`                            | 机器人的生命值                          |
| 等级（`level`）     | `uint`                            | 机器人的等级                           |
| 攻击状态（`attack`）  | `bool`                            | 机器人是否进行攻击                        |
| 信息丢失（`missing`） | `bool`                            | 当前的机器人信息是否是我方观察单位实时更新的有效信息       |

### PlaneCoordinate（平面坐标）

| 属性     | 类型       | 描述         |
| ------ | -------- | ---------- |
| 横坐标（x） | `double` | 世界坐标系中的横坐标 |
| 纵坐标（y） | `double` | 世界坐标系中的纵坐标 |

设计这个数据结构的出发点在于在当前的决策逻辑中几乎所有的坐标操作都只涉及到横坐标和纵坐标，而其它部分的数据处理极其繁琐，涉及到很多的重复代码。所以希望通过这个数据结构进行数值的传递。

该数据结构支持向量的加减和点乘，重载了相关的运算符号，并带有模长计算方法，可以很方便地进行各种运算。

与此同时，该数据结构可以很方便地转换为对应的消息（message），且可以直接转换为（`geometry::msg::PoseStamped`）。

### Area（区域）

地形和建筑的父类。

| 属性            | 类型                             | 描述              |
| ------------- | ------------------------------ | --------------- |
| 顶点集（vertices） | `std::vector<PlaneCoordinate>` | 一个由多个顶点构成的封闭多边形 |

### Terrain（地形）

目前没有进一步添加其他属性。

### Architecture（建筑）

| 属性            | 类型        | 描述     |
| ------------- | --------- | ------ |
| 生命值（hp）       | `uint`    | 建筑的生命值 |
| 阵营（`faction`） | `faction` | 建筑所属阵营 |

ChessboardHandle还拥有一系列的方法，可以方便地获取赛场中不同对象的状态信息。

```c++
inline std::shared_ptr<Robot> friend_robot(const uint& id);
inline std::shared_ptr<Robot> enemy_robot(const uint& id);
inline std::shared_ptr<Architecture> friend_outpost();
inline std::shared_ptr<Architecture> enemy_outpost();
inline std::shared_ptr<Architecture> friend_base();
inline std::shared_ptr<Architecture> enemy_base();
```

需要补充说明的是，在这段时间的更新中，增加了timestamp属性，用于不同作战单位间信息的传递和更新。

#### 1.1.2 Prism类说明

```c++
class Prism {
public:
    Prism() {}
    Prism(const iw_interfaces::msg::Prism& msg);

    std::unordered_map<std::string, double> parameters;
    Self self;
    Track track;
    Game game;
    Call call;

    void update_from_message(const iw_interfaces::msg::Prism& msg);

    iw_interfaces::msg::Prism to_message();
};
```

`Prism`类承载所有和作战单位本身有关的信息，包括参数`parameters`，自身信息`self`，追踪信息`track`，比赛信息`game`和场外呼叫信息`call`。不同信息的定义如下：

```c++
class Self {
public:
    uint id;                // 自身id
    uint hp;                // 自身血量
    uint ammo;              // 自身弹药量
    uint shooter_heat;      // 自身枪口热量
    PoseStamped pose;       // 自身位姿
};

class Track {
public:
    bool tracking;          // 是否正在追踪
    uint id;                // 追踪对象id
    uint hp;                // 追踪对象hp
    PoseStamped pose;       // 追踪对象位姿
};

class Game {
public:
    bool game_start;            // 比赛是否开始
    bool buy_ammo_ordered;      // 购弹指令是否被发出
    uint coins;                 // 金币数量
    uint projectile_allowance;  // 可发弹数量
};

class Call {
public:
    PlaneCoordinate target;    // 场外呼叫给出的指定目标
    bool is_called;            // 是否被呼叫
};
```

通过Prism对象，节点可以更加轻松且有逻辑地掌握，更新和自身相关的实时变化的信息。

#### 1.1.3 从对象到消息

`ChessboardHandle`类和`Prism`类都拥有`update_from_message`方法和`to_message`方法，可以很方便地完成于其对应消息类型的互相转换。

### 1.2 iw\_interfaces 信息组件的消息表示

包含所有information\_widgets中定义的组件的必要消息表示。`ChessboardHandle`类和`Prism`类，以及`Robot`，`Terrain`，`architecture`类，都可以通过各自的`update_from_message`方法和`to_message`方法完成和这些消息表示的互相转化。

### 1.3 sensing\_unit 感知单元

将所有能够传递给机器人的消息整理归纳进`ChessboardHandle`对象和`Prism`对象中，随后将它们转换成对应的消息表示，以50Hz的频率发布出来，其话题名称分别为`"rm_decision/chessboard"`和`"rm_decision/prism/<自身id>"`。于是，决策层只需要持有接收相应消息的订阅器，就可以获得实时更新的信息。

## 2. 动作层

涉及的功能包包括`navigator`和`navigator_interfaces`。

### 2.1 navigator说明

接收`navigator_interfaces::msg::Navigate`的消息类型，话题名称为"to\_navigator"。

`Navigate`消息包括`geometry_msgs::msg::PoseStamped`类型的机器人的位姿和布尔值`instant`。`navigator`接收该消息后，将调用下层的导航组件使机器人到达指定地点。根据`instant`值的真假，机器人将决定执行接收到的新的导航信息时是取消之前的导航动作立即执行，还是完成之前的导航动作之后再执行接收到的新动作。

理论上来说，设计决策层时，只需要了解以上信息就可以利用状态层和动作层的组件完成对上下层接口的对接。只需要充分利用好不同组件的特性，就可以更有逻辑和条理地利用场上信息作出决策。
