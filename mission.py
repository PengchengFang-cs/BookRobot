"""把四块积木按顺序接起来。这里就是整个任务的大脑。"""

from config import SCAN_ANGLE_RAD, SCAN_COUNT


def run_one_fruit(fruit, vision, navigation, arm, say=print):
    say(f"开始找 {fruit}")

    target_map = None
    for direction in range(SCAN_COUNT):
        target_map = vision.find(fruit, frame="map")
        if target_map is not None:
            break
        if direction + 1 < SCAN_COUNT:
            say("这一面没有，转九十度继续看")
            navigation.spin(SCAN_ANGLE_RAD)

    if target_map is None:
        say("四个方向都没有找到")
        navigation.go_home()
        return False

    say("看到了，现在走到水果前面")
    navigation.go(navigation.approach_pose(target_map))

    # 走动后再看一次；如果太近看不到，就用刚才记住的地图坐标。
    target_base = vision.find(fruit, frame="base_link")
    if target_base is None:
        target_base = navigation.map_point_to_base(target_map)

    say("开始用 MoveIt 规划抓取")
    arm.pick(target_base)
    say("拿到了，回到原点")
    navigation.go_home()
    arm.drop()
    say("任务完成，可以再说一种水果")
    return True
