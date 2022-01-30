# coding: UTF-8

# https://qiita.com/shizen-shin/items/b1d1f1944acf868b476b

#import モジュール名
    #import 親クラスを定義したモジュール名（.pyの前まで）
    #変数へのアクセスはモジュール名.変数名

#from モジュール名 import 変数名
    #モジュールの中から変数名を指定して取り込む
    #変数へのアクセスは変数名のみ 

# クラスの定義
    #class クラス名:

class Item:
    pass

#インスタンス生成
    #クラス名()
    #基本的に変数に代入する

item1 = Item()

#インスタンス変数のセット
    #インスタンス.インスタンス変数名 = セットする値

item1.name = 'sord'
item1.price = 5000
item1.special_skill = 'high'

#インスタンス変数の呼び出し
print(item1.name)
print(item1.price)
print(item1.special_skill)



#クラスの定義
    #class クラス名

class User:

    #__init__メソッド
        #def __init__(self, 引数1, 引数2,,,):
        #self必須
        #self=インスタンス自身
        #インスタンスを生成したときに自動で実行されるメソッド、初期値のセットなどに使う
        #__init__メソッドを呼び出したインスタンス自体が自動的にselfに入る
        #インスタンス変数名「name」を作り、引数で渡された値をセット

    def __init__(self, name):
        self.name = name

    #メソッドの作成
        #def メソッド名(self, 引数,,,):
        #self 必須

    def init_message(self):
        print('インスタンスを生成しました\n')


    def hello(self, prefecture):
        print('こんにちは、{}の{}さん。'.format(prefecture, self.name))

#インスタンスの生成
    #インスタンス = クラス名()
    #引数は__init__で設定した引数に合わせる
    #__init__メソッドが自動的に実行される

user1 = User('SHIBAYAMA')

#メソッド呼び出し
    #インスタンス.メソッド名(引数,,,)

user1.init_message()
user1.hello('hokkaido')
