---
title: java-oop-pro
createTime: 2025/12/27 14:44:18
permalink: /notes/Java/n3m7pseu/
---



## static

> 可以修饰成员变量和成员方法

1. **成员变量：**有static修饰的是**类变量**，没有static修饰的是**实例变量**。

- 类变量：属于类，与类一起加载一次，在内存中只有一份，可以被类和所有对象共享。

- 实例变量：属于对象，每个对象中都有一份，只能用对象访问。

- 类变量的应用：统计对象创建的个数，可以给类设置一个全局变量，即类变量，每创建一个对象就加一。
==一般static前面都加上public。==

2. **成员方法：**有static修饰的是**类方法**，没有static修饰的是**实例方法。**

- 类方法：属于类，直接用类名就可以访问。

-  实例方法：属于对象。

- 类方法的应用：工具类中的方法都是类方法，比如一个生成验证码的方法，在登录类和注册类里可以直接使用工具类的类名调用，提高代码复用性。
==工具类没有必要创建对象，纯粹为了调用方法，所有建议把工具类的构造器设计为私有，这样就不可以创建对象了。==

3. **代码块：**

-  静态代码块：static{}，类**加载**时自动执行，由于类只加载一次，所以静态代码快也只会执行一次。
- 实例代码块：{}，**创建**对象时会执行，并在**构造器之前**执行，创建多个对象会执行多次。

4. **单例设计模式：**
- 设计模式：一个问题的最优解法。

- 单例设计模式：确保一个类只有一个对象。首先把类构造为私有，然后定义一个类变量记住类的一个对象，最后定义一个类方法返回对象。

  ```java
  public class A {
      // 2.定义一个类变量p来记住一个对象
      private static A a = new A();
      
      // 1.使构造器私有化
      private A(){}
      
      // 3.定义一个类方法返回对象
      public static A getObject(){
          return a;
      }
  }
  ```
  
## 继承

>   public class B extends A{}

  子类继承父类的所有非私有成员。

这边的图片丢了。（悲，当时没上传图床）

  

**继承的注意事项：**

- 权限修饰符：public、private、protected、缺省。

| 修饰符    | 在本类中 | 同一个包的其他类中 | 任意包的子类里 | 任意包的任意类里 |
| --------- | -------- | ------------------ | -------------- | ---------------- |
| private   | √        |                    |                |                  |
| 缺省      | √        | √                  |                |                  |
| protected | √        | √                  | √              |                  |
| public    | √        | √                  | √              | √                |

- 单继承：一个类只能继承一个直接父类。

==java不支持多继承，因为有可能两个父类里面有名字一样的方法。==

- Object类：java所有类的祖宗类，我们写的任何一个类都是object的子类或子孙类。
- 方法重写：子类可以写一个方法来覆盖父类的某个方法，名字一样。

```java
@Override // 加上Override帮你检查
public void print1(){
    // 重写时，访问权限必须大于等于父类，比如父类是public，那子类只能是public
    // 重写方法的返回值要和父类一样，或者范围更小
    // 私有方法不能被重写
    // 以上不是很重要
}
```

==方法重写的一个应用：toString()重写，toString()重写也可以自动生成。==

- 子类访问成员：可以通过super来访问父类的成员。

这边的图片丢了。（悲，当时没上传图床）

- 子类的构造器特点：默认先调用父类的无参构造器，再执行自己。

==子类的构造器必须要父类有无参构造器才行，不然会报错。==

==子类中可以用super()来调用父类的有参构造器，实现初始化。==

- this调用兄弟构造器，实现函数的重载。

这边的图片丢了。（悲，当时没上传图床）

==this()和super()不能同时出现。==

## 多态

- 对象多态：编译看左边，运行看右边。

- 行为多态：编译看左边，运行看左边。

```java
People p1 = new Teacher(); // 对象多态
p1.run(); // 行为多态，注意这里的run是重用的父类的run
// 但是p1不能调用子类的特有方法
```

==如何让父类类型的变量可以调用子列的特有方法？==

答：使用**强制类型转换**。

```java
Teacher t1 = (Teacher) p1; // 不报错
// 然后t1就可以调用
// 但是不可以下面这样，因为p1是teacher对象不是student
Student s1 = (Student) p1; // 会报错
```

强制类型转换之前可以先用**instanceof**判断对象类型。

```
if (p1 instanceof Student){
	Student s1 = (Student) p1;
}
else{
	Teacher t1 = (Teacher) p1;
}
```

## final和常量

可以修饰类、方法、变量。使得类**不能被继承**，方法不能被重用，变量**只能被赋值一次**。
例如：public final class、public final void、final int。
用static final修饰的成员变量。

```java
public static final String SCHOOL_NAME = "黑马程序员"。
```

## 抽象类

- 抽象类中不一定有抽象方法，但反之不然。

- 类有的东西抽象类都可以有。
- ==抽象类不能创建对象，仅作为一种特殊的父类，让子类继承。（重要）==
- 继承抽象类的子类，必须重写完抽象类的全部抽象方法，否则子类也是抽象类。

```java
// 抽象类
public abstract class A {
    public abstract void run(); // 抽象方法
}
```
下面是子类B继承了抽象类A。
```java
public class B extends A {
    @Override
    public void run(){
        System.out.println("重用run");
    }
}
```

抽象的应用：模板方法设计模式。

## 接口

> java提供了一个关键字interface，用这个关键字我们可以定义出一个特殊的结构：接口

```java
public interface 接口名{
	// 成员变量（默认常量）
	// 成员方法（默认抽象方法）
    // 只能有以上两个元素，不能有其他东西。
}
```

==接口内定义的成员变量默认为常量，定义的方法默认为抽象方法。==

- 实现接口的类叫实现类。

```java
// 实现类，必须重写多个接口的全部抽象方法
public class D implements A_jiekou, B_jiekou {
	// 对着implements按alt+enter自动生成以下Override
    @Override
    public void testA1() {
    }

    @Override
    public void testB1() {
    }

    @Override
    public void testB2() {
    }
}
```

案例：面相接口编程。

- JDK8新增的接口方法

  1. 默认方法：用default修饰。

  2. 私有方法（JDK9之后）：用private修饰。

  3. 静态方法：用static修饰。

- 接口的多继承~

## 内部类

> 一个类定义在另一个类的内部

```java
public class Outer{
	public class Inner{
	}
}
// 内部类如何创建对象
Outer.Inner in = new Outer().new Inner();
```

==在成员内部类的实例方法中，拿到当前外部类的对象，格式是：外部类名.this。==

- 静态内部类：有static修饰的内部类，属于外部类自己持有。

```java
public class Outer{
	public static class Inner{
	}
}
// 内部类如何创建对象
Outer.Inner in = new Outer.Inner();
```

- 局部内部类：鸡肋。
- 匿名内部类：特殊的局部内部类；所谓匿名，指的是程序员不需要为这个类声明名字。

```java
Animal a = new Animal(){
	@Override
	public static cry(){
		System.out.println("喵喵叫~");
	}
};
a.cry();
// 会自动创建Animal的一个子类，在子类里面重写cry();
// cry()是抽象类Animal的一个抽象方法
```

==匿名内部类在开发中通常是直接作为一个参数来传给方法的。==

```java
package nimingneibulei;

public class Test {
    public static void main(String[] args) {
        go(new Swimming(){
            @Override
            public void swim(){
                System.out.println("狗狗游得飞快");
            }
        });// 这里匿名内部类作为参数传递给go()方法，这里的内部类也是一个实现类
    }

    // 设计一个go()方法，接受Swimming接口的一切实现类，表示参加游泳比赛
    public static void go(Swimming s){
        System.out.println("-----------开始-----------");
        s.swim();
    }
}

// Swimming接口
interface Swimming{
    void swim();
}
```

## 枚举

> 枚举是一种特殊的类，有固定的对象数量

```java
public enum A {
    // 枚举类的第一行必须罗列枚举名称
    X, Y, Z;

    // 后面就和普通类一样
    private String name;

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }
}
```

- 关于枚举的用法

```java
public class Test {
    public static void main(String[] args) {

        // 枚举的构造器是私有的，不能对外创建对象
        // A a = new A();是不行的

        // 枚举类的第一行都是常量，记住的是枚举类的对象
        A a1 = A.X;
        System.out.println(a1);

        // 一些API
        A[] as = A.values(); // 拿到全部对象
        A a2 = A.valueOf("Z");
        System.out.println(a2.name()); // Z
        System.out.println(a2.ordinal()); // Z的索引
    }
}
```

- 抽象枚举

## 泛型

> 泛型，即“参数化类型”。以方法的定义为例，在方法定义时，将方法签名中的`形参的数据类型`也设置为参数（也可称之为类型参数），在调用该方法时再从外部传入一个具体的数据类型和变量。

- 自定义泛型类

```java
// 泛型类
public class MyArrayList<E> {
    public boolean add(E e){
        return true;
    }

    public E get(int index){
        return null;
    }
}
```

- 自定义泛型接口
```java
// 泛型接口
public interface Data<T>{
    void add(T t);
    ArrayList<T> getByName(String name);
}
```
- 自定义泛型方法

```java
// 泛型方法
public static <T> T test(T t){
    return t;
}
// 输入的是什么类型，返回的就是什么类型
```

==泛型不支持基本数据类型（比如int），只能支持对象类型（引用数据类型比如String）。==

