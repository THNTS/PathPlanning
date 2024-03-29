#Бабин Даниил, проект PathPlanning


На данный момент программа по данному на вход xml должна находить простой путь при помощи алгоритма А* и Дейкстры, как частного
случая А*.

Входные данные:
	В качестве аргумента программа получает на вход xml файл, в котром задаются сначала
параметры карты: ширина, высота, стартовая и конечная точка и сама карта в виде клеток, по
которым программа может "ходить" (обозначаются 1-ей) или нет (обзначаются 0-ём). 
	Затем задаются параметры поиска пути:
		
		-searchtype - выбор алгоритма поиска (на данный момент Дейкстра и А*)
		

		-hweight - задаёт вес эвристики для алгоритма поиска
		
		-allowdiagonal - может ли путь идти по диагоналям.
		
		-cutcorners - может ли путь огибать угол препятствия через вершину, которая
		представляется как угол препятствия.

		-allowsqueeze - может ли путь проходить через точку, по диагонали у которой
		две клетки с препятствиями, а по остальным двум можно ходить. 	
    
    
A* пошагово просматривает все пути, ведущие от начальной вершины в конечную, пока не найдёт минимальный. Как и все информированные алгоритмы поиска, он просматривает сначала те маршруты, которые «кажутся» ведущими к цели. От жадного алгоритма, который тоже является алгоритмом поиска по первому лучшему совпадению, его отличает то, что при выборе вершины он учитывает, помимо прочего, весь пройденный до неё путь. Составляющая g(x) — это стоимость пути от начальной вершины, а не от предыдущей, как в жадном алгоритме.

В начале работы просматриваются узлы, смежные с начальным; выбирается тот из них, который имеет минимальное значение f(x), после чего этот узел раскрывается. На каждом этапе алгоритм оперирует с множеством путей из начальной точки до всех ещё не раскрытых (листовых) вершин графа — множеством частных решений, — которое размещается в очереди с приоритетом. Приоритет пути определяется по значению f(x) = g(x) + h(x). Алгоритм продолжает свою работу до тех пор, пока значение f(x) целевой вершины не окажется меньшим, чем любое значение в очереди, либо пока всё дерево не будет просмотрено. Из множества решений выбирается решение с наименьшей стоимостью.

Чем меньше эвристика h(x), тем больше приоритет, поэтому для реализации очереди можно использовать сортирующие деревья.

Множество просмотренных вершин хранится в closed, а требующие рассмотрения пути — в очереди с приоритетом open. Приоритет пути вычисляется с помощью функции f(x) внутри реализации очереди с приоритетом. 
