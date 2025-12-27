---
title: common-APIs
createTime: 2025/12/27 14:44:18
permalink: /notes/Java/x1msoa5y/
---
[TOC]



## Object类

1. `toString()`

​	输出对象地址，可以重写后输出对象的内容。

2. `equals(Object o)`

​	输出地址是否相同，可以重写后输出对象的内容是否一致。

3. `clone()`

   将一个对象克隆给另一个新创建的对象，需要先重写，具体用法到时候自己搜。

- 浅克隆：拷贝出的对象，与原对象中的数据一模一样（引用类型拷贝的只是地址）。

- 深克隆：拷贝出的对象，基本类型的数据直接拷贝，字符串拷贝地址，但是对象不是拷贝地址，而是创建新对象。

## Objects类

1. `equals(Object a, Object b)`
   先做非空判断，再比较两个对象。用法是`Objects.equals(s1, s2)`，尽量不用`s1.equals(s2)`。因为s1可能为null。
2. `isNull(Object obj)`
   判断对象是否为null，为null返回true，反之。
3. `nonNull(Object obj)`
   判断对象是否为null，不为null则返回true，反之。

## 包装类

> 把基本类型的数据包装成类。

| 基本数据类型 | 对应的包装类（引用数据类型） |
| ------------ | ---------------------------- |
| byte         | Byte                         |
| short        | Short                        |
| int          | Integer                      |
| long         | Long                         |
| char         | Character                    |
| float        | Float                        |
| double       | Double                       |
| boolean      | Boolean                      |

用法如下：
```java
// 把12包装为一个Integer类
Integer a1 = Integer.valueof(12);

// 不能这么写，已经过时了
Integer a2 = new Integer(12);

// 或者可以简写，自动装箱机制
Integer a3 = 12;

// 自动拆箱，把包装后的拆分为基本类型
int a4 = a3;
```

==泛型和集合不支持基本数据类型，只支持引用数据类型。==

- 包装类的其他操作：

```java
// 把基本类型的数据转换字符串
Integer a = 23;
String rs1 = Integer.toString(a); // 23

// 把字符串类型的数值转换成对应的基本类型
String ageStr = "29";
int ageI = Integer.valueof(ageStr); // 29
String score = "99.5";
double ageD = Double.valueof(score); // 99.5
```

## StringBuilder类

> 它是一个可变的字符串操作类，允许我们对字符串进行高效的操作。

1. 初始化

```java
// 无参构造函数创建StringBuilder对象
StringBuilder sb1 = new StringBuilder();

// 带有初始容量参数的构造函数创建StringBuilder对象
StringBuilder sb2 = new StringBuilder(20);

// 创建时存入
StringBuilder sb3 = new StringBuilder("itheima");
```

2. 基本操作

```java
StringBuilder sb = new StringBuilder("Hello");
StringBuilder sb1 = new StringBuilder();

// 追加字符串
sb.append(", Java!");
System.out.println(sb.toString());  // 输出：Hello, Java!

// 插入字符串
sb.insert(7, "World, ");
System.out.println(sb.toString());  // 输出：Hello, World, Java!

// 删除字符或子串
sb.deleteCharAt(5);
System.out.println(sb.toString());  // 输出：Hello World, Java!

// 替换字符或子串，原先字符串的位置6-11被替换
sb.replace(6, 11, "Java Programming");
System.out.println(sb.toString());  // 输出：Hello Java Programming, Java!

// 通过sb,toString()转换为String
```

==StingBuilder拼接效率很高。==

## StringJoiner类

> StringJoiner是Java8新出的用于处理字符串拼接的工具类，可以让你的代码看起来更优雅，不拉跨。

```java
// 三个参数分别是间隔符，起始符，终结符
StringJoiner s = new StringJoiner(", ", "[", "]");
s.add("12");
s.add("34");
s.add("56");
// 输出结果是[12, 34, 56]
```

==StringJoiner的拼接是add，且只能拼接字符串。==

## Math类（不太常用）

```java
Math.abs(int/double a); // 绝对值
Math.ceil(double a); // 向上取整
Math.floor(double a); // 向下取整
Math.round(float a); // 四舍五入
Math.max(int a, int b);// 比大小
Math.pow(double a, double b); // a的b次方
Math.random(); // 取随机数[0.0, 1.0)左闭右开
```

## System类

> 主要用于获取系统的属性数据和其他操作。

```java
System.exit(0); // 认为终止虚拟机
System.currentTimeMillis(); // 返回1970.1.1 0:0:0开始到现在的毫秒值，long类型
```

## Runtime类

> 代表程序所在的运行环境，是一个单例类。

```java
Runtime r = Runtime.getRuntime(); // 返回与当前Java应用程序有关联的运行时对象
r.exit(); // 终止当前运行的虚拟机
r.availableProcessors(); // 虚拟机能够使用的处理器数, int类型
r.totalMemory(); // 返回Java虚拟机中的内存总量,单位是字节，long类型
r.freeMemory(); // 返回Java虚拟机中的可用内存量
r.exec(); // 启动某个程序，并返回代表该程序的对象，括号里面填启动路径

// 启动qq然后关闭
Process p = r.exec("D:\\QQ.exe");
Thread.sleep(5000);// 暂停五秒
p.destroy();
```

## BigDecimal类

> 解决浮点型运算时，出现的结果失真问题。

```java
double a = 1.2323;
double b = 3.2323;

// 字符串类型转换为BigDecimal类型
BigDecimal a1 = BigDecimal.valueOf(a);
BigDecimal b1 = BigDecimal.valueOf(b);

BigDecimal c1 = a1.add(b1); // +
BigDecimal c2 = a1.multiply(b1); // *
BigDecimal c3 = a1.subtract(b1); // -
BigDecimal c4 = a1.divide(b1, 2, RoundingMode.HALF_UP); // /保留两位，四舍五入

double  cc1 = c1.doubleValue(); // 转换为double
```

## 日期和时间

> 了解一下吧，现在基本不用了

- Date

```java
Date d = new Date();
System.out.println(d); // 输出日期
System.out.println(d.getTime()); // 输出毫秒值

long time = d.getTime();
time += 2*1000; // 时间加上2s
Date d2 = new Date(time); // 毫秒值转换为日期
System.out.println(d2);
```

- SimpleDateFormat简单日期格式化

```java
Date d = new Date();
long time = d.getTime();

// 相当于定义了一个格式
SimpleDateFormat t = new SimpleDateFormat("yyyy年MM月dd日，HH:mm:ss EEE a");
String ff = t.format(d);// 把date类型的对象传入t格式中，也可以传long类型的毫秒值
System.out.println(ff);
```

- SimpleDateFormat把字符串时间解析为日期对象（parse）

```java
String dateStr = "2022-12-12 12:12:11";
// 解析的格式必须与指定的格式一样
SimpleDateFormat t1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
Date d1 = t1.parse(dateStr);
System.out.println(d1);
```

- Calender日历对象

```java
// 创建日历对象
Calendar c = Calendar.getInstance();

// 获取日历对象中的某个信息
int year = c.get(Calendar.YEAR);
System.out.println(year); // 2025

// 获取日历中记录的日期对象
Date d = c.getTime();
System.out.println(d);

// 拿到毫秒值
long t = c.getTimeInMillis();
System.out.println(t);

// 修改日历中的某个信息
c.set(Calendar.MONTH, 9);
int m = c.get(Calendar.MONTH);
System.out.println(m);// 9

// 为某个信息增加或减少天，这里第二个参数表示增加了10天
c.add(Calendar.MONTH,10);
m = c.get(Calendar.MONTH);
System.out.println(m);
```

## JDK8后新增的时期和时间

> 这个要会

LocalDate：年 月 日

LocalTime：时 分 秒 纳秒

LocalDateTime：包含以上两个，更常用

- 下面以LocalDate为例子。

```java
// 获取本地日期，ld是不可变对象
LocalDate ld = LocalDate.now(); // 2025-02-07

// 获取LocalDate中的详细信息
int year = ld.getYear(); // 哪一年
int dayOfWeek = ld.getDayOfYear(); // 一年中的第几天
int dayOfYear = ld.getDayOfWeek().getValue(); // 星期几

// with直接修改某个信息，ld是不可变对象，所以修改后的必须重新赋值给新的LocalDate对象
LocalDate ld2 = ld.withYear(2077);
System.out.println(ld2); // 2077-02-07

// plus和minus把某个信息增加或者减少多少
LocalDate ld3 = ld.plusDays(1);
System.out.println(ld3); // 2025-02-08
LocalDate ld4 = ld.minusDays(1);
System.out.println(ld4); // 2025-02-06

// 获取指定的LocalDate对象
LocalDate ld5 = LocalDate.of(2077, 1, 2);
System.out.println(ld5); // 2077-01-02

// 比较两个日期对象 equals, isBefore, isAfter
System.out.println(ld5.equals(ld)); // false
System.out.println(ld5.isAfter(ld)); // true

// 此外，LocalDateTime可以转换为LocalDate和LocalTime，直接赋值就行
// 两个小的也可以用LocalDateTime.of()合并成大的
LocalDateTime ldt = LocalDateTime.of(ld, lt);
```

- DateTimeFormatter代替SimpleDateFormat管理格式

```java
// 创建一个格式对象
DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy年MM月dd日 HH:mm:ss");

// 对时间进行格式化
LocalDateTime now = LocalDateTime.now();
String rs = formatter.format(now);
System.out.println(rs); // 2025年02月08日 22:17:14

// 或者调用LocalTimeDate的方法
String  rs1 = now.format(formatter);
System.out.println(rs1); // 2025年02月08日 22:17:14

// 解析时间
String dateStr = "2077年12月13日 12:12:11";
LocalDateTime ldt = LocalDateTime.parse(dateStr, formatter);
System.out.println(ldt); // 2077-12-13T12:12:11
```

- Period计算日期间隔，用来计算两个LocalDate对象

懒得写了，就是可以用Period.between()返回一个Period对象，然后用getYears()等等方法来获取具体信息。

- Duration计算持续时间，用来计算两个LocalTime、LocalDateTime、Instant等

与上面那个差不多。

## Arrays类

> 操作数组的工具类

```java
// 返回数组内容
int [] arr = {1, 2, 3, 4};
System.out.println(Arrays.toString(arr)); // [1, 2, 3, 4]

// 拷贝数组，左闭右开
int [] arr2 = Arrays.copyOfRange(arr, 1, 3);
System.out.println(Arrays.toString(arr2));  // [2, 3]

// 拷贝数组，新数组定长
int [] arr3 = Arrays.copyOf(arr, 10);
System.out.println(Arrays.toString(arr3)); // [1, 2, 3, 4, 0, 0, 0, 0, 0, 0]

//setAll方法

// sort，默认从小到大
Arrays.sort(arr);
```

- 数组中存的对象如何排序？让类继承Comparable接口，重写compareTo方法。

```java
public class Stu implements Comparable <Stu>{
    private String name;
    private int age;
    private double score;

    @Override
    public int compareTo(Stu o) {
        // 小于返回负数，大于返回正数
        return this.age - o.age;
    }
```

还有一种方法是在sort方法中创建Comparator比较器接口的匿名内部类对象，自己制定比较规则。

```java
Stu[] students = new Stu[4];
students[0] = new Stu("yyx", 20, 100);
students[1] = new Stu("yyy", 23, 99);
students[2] = new Stu("xxx", 18, 70);
students[3] = new Stu("xxy", 29, 80);

// 匿名内部类
Arrays.sort(students, new Comparator<Stu>() {
    @Override
    public int compare(Stu o1, Stu o2) {
        return Double.compare(o1.getScore(), o2.getScore()); // 升序
    }
});

// 以上可以简化为如下
Array.sort(students, (Stu o1, Stu o2) -> {
    return Double.compare(o1.getScore(), o2.getScore());
});

// 还可以继续简化
Array.sort(students, ( o1, o2) -> Double.compare(o1.getScore(), o2.getScore()));
```

## Lambda表达式

> 用于简化匿名内部类的代码写法，只能简化函数式接口的匿名内部类

```java
public class lambdaDemo {
    public static void main(String[] args) {
        
        // 形式：(被重写方法的形参列表) -> {被重写方法的方法体代码}
        Swimming a = () -> {
            System.out.println("游泳");
        };
        a.swim();
    }
}

interface Swimming{
    void swim();
}
```

## 方法引用进一步简化Lambda

静态方法引用

实例方法引用

特定类型方法的引用

构造器引用

==太能简化了。。。。。。这边省略不写了。。。。。==

## 正则表达式

> 校验数据格式是否合法或者在一段文本中查找满足要求的内容

- 基本语法

这边的图片丢了。（悲，当时没上传图床）

- 校验数据格式是否合法示例（电话号码与邮箱的匹配）

```java
// 一个电话号码的匹配方法
public static void checkPhone(){
    Scanner sc = new Scanner(System.in);
    while (true) {
        System.out.println("输入电话号码");
        String phone = sc.nextLine();
        // 正则表达式中不能有空格，否则会干扰匹配
        if (phone.matches("1[3-9]\\d{9}")){
            System.out.println("格式正确");19
            break;
        }
        else {
            System.out.println("不正确重来");
        }
    }
}
```

```java
// 一个邮箱的匹配方法
public static void checkEmail(){
    Scanner sc = new Scanner(System.in);
    while (true) {
        System.out.println("输入邮箱");
        String phone = sc.nextLine();

        // 邮箱分为三段，@和.来分三段
        if (phone.matches("\\w{2,}@\\w{2,20}(\\.\\w{2,10}){1,2}")){
            System.out.println("格式正确");
            break;
        }
        else {
            System.out.println("不正确重来");
        }
    }
}
```

- 在一段文本中查找满足要求的内容示例（爬取电话和邮箱）

```java
public static void method1(String data){
    // 定义爬取规则
    String regex = "(1[3-9]\\d{9})|(\\w{2,}@\\w{2,20}(\\.\\w{2,10}){1,2})";

    // 把正则表达式封装成一个pattern对象
    Pattern pattern = Pattern.compile(regex);

    // 通过pattern对象去获取查找内容的匹配器对象
    Matcher matcher = pattern.matcher(data);

    // 开始爬取信息
    while (matcher.find()){
        String rs = matcher.group();
        System.out.println(rs);
    }

}
```

- 替换操作，获取人名

```java
// 替换
String s1 = "蔡徐坤23432df陈立农df34df范丞丞dfdf私募";
System.out.println(s1.replaceAll("\\w+", "-")); // 蔡徐坤-陈立农-范丞丞-私募

// 获取人名
String s2 = "蔡徐坤23432df陈立农df34df范丞丞dfdf私募";
String[] name =  s2.split("\\w+");
System.out.println(Arrays.toString(name)); // [蔡徐坤, 陈立农, 范丞丞, 私募]
```

## 异常

- 自定义异常
- 异常处理
