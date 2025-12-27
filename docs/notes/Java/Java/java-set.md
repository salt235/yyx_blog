---
title: java-set
createTime: 2025/12/27 14:44:18
permalink: /notes/Java/abkbhoia/
---
# Java集合框架体系

> **Java 集合可分为 Collection 和 Map 两大体系**

- Collection接口：用于存储一个一个的数据，也称单列数据集合
  - List子接口：用来存储有序的、可以重复的数据（主要用来替换数组，"动态"数组）
    - 实现类：ArrayList(主要实现类)、LinkedList、Vector
  - Set子接口：用来存储无序的、不可重复的数据（类似于高中讲的"集合"）
    - 实现类：HashSet(主要实现类)、LinkedHashSet、TreeSet
- Map接口：用于存储具有映射关系“key-value对”的集合，即一对一对的数据，也称双列数据集合
    - HashMap(主要实现类)、LinkedHashMap、TreeMap、Hashtable、Properties

[TOC]

## Collection集合的常用方法

```java
// collection是接口
Collection<String> c = new ArrayList<>();
// add添加元素
c.add("123");
// clear清空
c.clear();
System.out.println(c.isEmpty()); // true
System.out.println(c.size()); // 0
// contains是否包含
c.add("123");
System.out.println(c.contains("123")); // true
// remove删除，优先删除第一个出现的
c.remove("123");
System.out.println(c); // []
// 集合转数组
Object[] arr = c.toArray();
System.out.println(Arrays.toString(arr)); // []
// 把一个集合的全部数据导入另一个集合
Collection<String> c1 = new ArrayList<>();
c1.add("123");
Collection<String> c2 = new ArrayList<>();
c2.add("344");
c1.addAll(c2);
System.out.println(c1); // [123, 344]
```

## Collection集合的遍历

- 迭代器遍历

```java
Collection<String> c = new ArrayList<>();
c.add("yyx");
c.add("yyy");
c.add("xxx");

Iterator<String> it = c.iterator();
while (it.hasNext()){
    String ele = it.next();
    System.out.println(ele);
}
```

- 增强for

```java
// 输入c.for按回车自动生成
for (String ele : c) {
    System.out.println(ele);
}
```

- Lambda表达式

```java
c.forEach(new Consumer<String>() {
    @Override
    public void accept(String s) {
        System.out.println(s);
    }
});
// 继续简化
c.forEach((String s) -> {
        System.out.println(s);
});
// 继续简化
c.forEach(s -> {
    System.out.println(s);
});
// 继续简化
c.forEach(System.out::println);
```

## List集合

> 有序 可重复 有索引

- 常用方法

```java
List<String> list = new ArrayList<>(); // 一行经典代码
list.add("yyx");
list.add("yyy");
list.add("xxx");

list.add(1,"123"); // [yyx, 123, yyy, xxx]
list.remove(1); // [yyx, yyy, xxx]
System.out.println(list.get(2)); // xxx
list.set(1, "yyyyyyyy"); // 修改指定索引
```

==ArrayList 同时满足 Collection 和 List 的接口规范，因此可以作为两者的实现类。==

- ArrayList（顺序表）

- LinkedList（双链表）

```java
// LinkedList模拟queue
LinkedList<String> queue = new LinkedList<>();

// 入队
queue.addLast("1");
queue.addLast("2");
queue.addLast("3");
queue.addLast("4");
System.out.println(queue); // [1, 2, 3, 4]

// 出队
queue.removeFirst();
System.out.println(queue); // [2, 3, 4]
```

==这里不能用Collection创建，因为多态的特点，不能使用子类的特有方法，这里创建队列的话需要LinkedList的特有方法。==

```
// LinkedList模拟stack
LinkedList<String> stack = new LinkedList<>();

// 压栈
stack.push("1");
stack.push("2");
stack.push("3");
stack.push("4");
System.out.println(stack); // [4, 3, 2, 1]

// 出栈
stack.pop();
System.out.println(stack); // [3, 2, 1]
```

## Set集合

> 无序 不重复 无索引

分为HashSet、LinkedHashSet（有序）、TreeSet（排序）。

- HashSet

就是哈希表，增删改查的性能都比较好，只不过是无序的不可索引不可重复。

```java
Set<Integer> set = new HashSet<>(); // 一行经典代码
```

- LinkedHashSet

在HashSet的基础上，利用双链表来记录每个元素前后元素的位置，所有可以有序。

- TreeSet

最核心的特点是可排序，基于红黑树。

对于自定义的对象，TreeSet无法排序，此时要进行compareTo的重写或者匿名内部类中对compare重写。

- 并发修改异常问题

1. 使用迭代器时，用迭代器自身的remove方法来删除。
2. 使用fori遍历时，可以倒着遍历，或者遍历时i--。
3. 使用增强for时，无法解决这个问题。
4. 使用foreach时，也无法解决这个问题。

## Collections工具类

- 可变参数

使用`类型... 参数名`声明，例如：

```java
void printValues(String... values) {
    // 方法内将values视为数组处理
    for (String s : values) {
        System.out.println(s);
    }
}
```

**自动封装为数组**：传入的参数会被隐式转换为数组，例如`printValues("A", "B")`等价于`printValues(new String[] {"A", "B"})`。

**位置限制**：可变参数必须是方法参数的==最后一个==，例如`void method(int a, String... values)`合法，但`void method(String... values, int a)`不合法。

**可接受空参数**：可传入0个参数（即不传）、单独参数或数组，例如`printValues()`或`printValues(null)`（需处理`null`情况）。

**数量限制**：参数列表只能有一个可变形参。

- collections

```java
List<String> list = new ArrayList<>();
// 批量添加
Collections.addAll(list, "yx", "yyx", "xxx"); /// [yx, yyx, xxx]
// 打乱顺序
Collections.shuffle(list); // 每次运行打乱的都不一样，随机的
// 对List集合中的元素进行升序排序
Collections.sort(list); // [xxx, yx, yyx]
```

## Map集合

> 双列集合

- 常用方法

```java
Map<Integer, String> map = new HashMap<>();
map.put(100, "手表");
map.put(102, "Java");
map.put(103, "Python");
System.out.println(map); // {100=手表, 102=Java, 103=Python}
// map.size();
// map.clear();
// map.isEmpty();
System.out.println(map.get(100)); // 手表
// 把手表删了，返回手表对应的值
map.remove(100);
// 是否包含键
System.out.println(map.containsKey(102)); // true
// 是否包含值
System.out.println(map.containsValue("Java")); // true
// 获取map的所有键
Set<Integer> keys = map.keySet();
// 获取map的所有值
Collection<String> values = map.values();
// 把其他Map集合的数据倒入到自己的集合中来，相同的键会覆盖
Map<Integer, String> map2 = new HashMap<>();
map2.put(102, "jjjjjjava");
map.putAll(map2);
System.out.println(map); // {102=jjjjjjava, 103=Python}
```

- 遍历

1. 通过键来遍历

```java
// 通过键来遍历
Set<String> keys = map.keySet();
for (String key : keys) {
    double value = map.get(key);
    System.out.println(key + "->" + value);
}
```

2. 通过键值对（太复杂不学了）
3. 通过Lambda（方便）

```java
// 通过lambda表达式
map.forEach((k, v) -> {
    System.out.println(k + "->" + v);
});
```

- HashMap

无序 不重复 无索引

- LinkedHashMap

有序 不重复 无索引

- TreeMap

排序 不重复 无索引

- 集合的嵌套

## Stream流

> 经典白学

- Stream流的获取

```java
// 获取List集合的stream流
List<String> list = new ArrayList<>();
Collections.addAll(list, "yyx", "yyy", "xxx");
Stream<String> stream = list.stream();

// 获取Set集合的stream流
Set<String> set = new HashSet<>();
Collections.addAll(set, "yyx", "yyy", "xxx");
Stream<String> stream1 = set.stream();
stream1.filter(a -> a.contains("x")).forEach(a -> System.out.println(a));

// 获取Map集合的stream流
Map<String, Double> map = new HashMap<>();
map.put("yyx", 100.0);
map.put("yyy", 101.0);
map.put("xxx", 102.0);
// 获取键
Set<String> keys = map.keySet();
Stream<String> stream2 = keys.stream();
// 获取值
Collection<Double> values = map.values();
Stream<Double> stream3 = values.stream();
// 同时获取键和值
Set<Map.Entry<String, Double>> entries = map.entrySet();
Stream<Map.Entry<String, Double>> stream4 = entries.stream();
stream4.filter(e -> e.getKey().contains("x"))
        .forEach(e -> System.out.println(e.getKey() + "->" + e.getValue()));

// 获取数组的stream流，两种方法
String[] names = {"yyx", "yyy", "xxx"};
Stream<String> stream5 = Arrays.stream(names);
Stream<String> stream6 = Stream.of(names);
```

- Stream流提供的中间方法

filter过滤

sorted排序

limit只选择前几个来操作

skip跳过前几个

distinct去重

map映射（加工）

concat流合并

```java
// 基本数据集合
List<Double> list = new ArrayList<>();
Collections.addAll(list, 99.0, 100.0, 78.1);
// 筛选大于90的 并排序输出
list.stream().filter(s -> s > 90.0).sorted().forEach(s -> System.out.println(s));
```

```java
// 对象集合
List<Student> students = new ArrayList<>();
Student s1 = new Student("yyx", 22, 100);
Student s2 = new Student("yyy", 21, 90);
Student s3 = new Student("xxx", 19, 98);
Student s4 = new Student("xxx", 20, 95);
Collections.addAll(students, s1, s2, s3, s4);
// 筛选大于20岁的，按照年龄降序输出
students.stream().filter(s -> s.getAge() > 20).sorted((o1, o2) -> o2.getAge() - o1.getAge()).forEach(System.out::println);
// 只选择成绩最高的两个降序输出
students.stream().sorted((o1, o2)-> Double.compare(o2.getScore(), o1.getScore())).limit(2).forEach(System.out::println);
// 跳过前一个，输出后面两个
students.stream().skip(1).forEach(System.out::println);
// 去除重复名字，输出
students.stream().map(s -> s.getName()).distinct().forEach(System.out::println);

```

==distinct去重复，如果需要自定义类型的对象，则需要重写hashCode和equals。==

```java
Stream<String> st1 = Stream.of("yyx", "yy");
Stream<String> st2 = Stream.of("xxx", "zz");
Stream<String> st3 = Stream.concat(st1, st2);
st3.forEach(System.out::println);
```

- Stream终结方法

forEach 遍历

count元素个数

max/min最大最小值

```java
List<Student> students = new ArrayList<>();
Student s1 = new Student("yyx", 22, 100);
Student s2 = new Student("yyy", 21, 90);
Student s3 = new Student("xxx", 19, 98);
Student s4 = new Student("xxx", 20, 95);
Collections.addAll(students, s1, s2, s3, s4);

// 找出分数最高的对象
Student student666 = students.stream()
    .max(((o1, o2) -> Double.compare(o1.getScore(), o2.getScore())))
    .get();
System.out.println(student666); // Student{name='yyx', age=22, score=100.0}
```

- 收集Stream流

>  把Stream流操作后的结果转回到集合或者数组中返回

==流只能收集一次==

collect 收集到List或者Set集合中

```java
// 把分数超过90分的收集到List或者Set集合中
List<Student> st1 =  students.stream()
    .filter(s -> s.getScore() > 95)
    .collect(Collectors.toList()); // 改为toSet可以存入Set中，但会去重
System.out.println(st1); 
```

collect 收集到Map集合中

```java
// 把分数超过90分的收集到一个新Map中
Map<String, Double> map =  students.stream()
    .filter(s -> s.getScore() > 95)
    .distinct()
    .collect(Collectors.toMap(Student::getName, s -> s.getScore()));
```

toArray 收集到数组中

```java
// 把分数超过90分的收集到一个数组中
Object[] arr = students.stream().filter(s -> s.getScore() > 95).toArray();
System.out.println(Arrays.toString(arr));
```
