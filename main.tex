\documentclass{article}
\usepackage{amsmath} % For \text, \lfloor, \rfloor, and other math environments
\usepackage{amssymb} % For various math symbols
\usepackage[utf8]{inputenc} % Standard for modern LaTeX documents
\usepackage{parskip} % To have space between paragraphs instead of indentation, closer to the image's style.
                     % If you prefer indentation, remove this line.

\begin{document}

In the disaster hit area random deployment of sensor nodes having limited battery power consumes a lot of energy for capturing data, local processing and transmission of data. Moreover, high deployment density is expected so that the failure of few sensors can’t affect the smooth working of the application. But high deployment density results into larger overlap in the coverage area of sensor nodes. Hence the node density and node deployment are the two challenging issues in the context of maximizing the area coverage and minimizing the energy consumption of the sensor nodes in a disaster hit area. Two different approaches (APP\_I and APP\_II) are proposed in the present work. APP\_I is a deterministic approach using grid-based deployment for the reduction of node density and execution time to complete the sensor deployment. APP\_I is made more realistic in APP\_II which is a non-deterministic approach using heuristic algorithm for obtaining optimal deployment and for the reduction of the execution time. The objective of both the approaches is to maximize the percentage of area coverage and to reduce the execution time to complete the sensor deployment in the target area.

\section*{Approach I}

APP\_I is elaborated in this section. It is a deterministic grid based deployment for WSNs. It considers two different phases of operation, global phase and local phase.

In this approach a system model is considered wherein a set of sensors ($N_s$) are randomly deployed within a specified target area. All the sensors are of the same type i.e. they all have the same sensing range ($R_s$) and communication range ($R_c$). Each sensor knows its position using GPS and the dimension of the target area. Each sensor can also detect any event within its coverage area. A base station (BS) is situated at the bank of the target area. BS also knows its position using GPS and the dimension of the target area. $R_c$ of BS is equal to $R_c$ of each sensor. Two mobile robots are considered for distributing and deploying sensors evenly in the target area to enhance area coverage and network performance. The global phase is executed by the BS as discussed in section~x whereas the local phase is executed by the mobile robot as discussed in section~y.

The BS divides the target area into NO\_LA number of location areas (LAs) depending upon the perception range (R) of robot and inserts NO\_LA number of records in a database (LA\_DB).
\[
\text{NO\_LA} = \lfloor \text{Size of target area} / \text{Perception range of robot} \rfloor
\]
The size of each LA correlates directly with the perception range of the mobile robot, ensuring that all sensors within a LA fall within the robot's perception range. Each LA is geometrically defined as a rectangular or square space, with dimensions tailored to fit entirely within the perception range of the mobile robot.

The BS deploys two mobile robots in the two different LAs to cater sensors that are already randomly deployed in those LAs evenly, inserts the deployment details of robot in a database (Robot\_DB).
It is assumed that each robot has the capacity to store 15 sensors and BS assigns 10 sensors (Stock\_RS) to each robot at the time of its deployment in a LA.

Each robot executes local phase after its deployment in a LA.
In the local phase robot divides the assigned LA into a few grids (NO\_G) and inserts NO\_G number of records in a database (Grid\_DB), each such grid is covered if a sensor is present at its centre.
\[
\text{NO\_G} = \lfloor \text{Perception range of robot} / \text{Perception range of sensor} \rfloor
\]
The robot identifies the sensors (NO\_S) which are within its perception range and inserts NO\_S number of records in a database (Sensor\_DB).


Both BS and robot know the number of permissible move ($\text{NO\_P}$) of robot within a LA during a local phase. Initially $\text{NO\_P}$ is $\text{NO\_G}$ at the time of starting a local phase by the robot. Robot reduces $\text{NO\_P}$ by 1 after visiting each grid within a LA during its local phase and stops the execution of the current local phase when $\text{NO\_P}$ is 0. It helps keep the robot movement within limit which in turn reduces energy consumption as discussed in section z due to robot movement.

The global phase is responsible for assigning location areas to individual mobile robots. This phase focuses on the macro-level coordination of robot deployment across the target area. Conversely, the local phase entails the execution of sensor redeployment by the mobile robots within their designated LAs. Here the emphasis shifts to micro-level operations, where the robots undertake the actual relocation of sensors to achieve maximal coverage within their assigned LAs.

\noindent\textbf{p. Format and size of databases}

\noindent\textbf{Format and size of LA\_DB:} $\text{NO\_Grid}$ is the number of covered grid in a particular LA. Initially the value of this attribute in all the records of $\text{LA\_DB}$ is 0.

\begin{center}
\begin{tabular}{|l|l|l|}
\hline
LA\_id & X-Y coordinate of the centre of LA & NO\_Grid \\
\hline
$\text{LA\_id}_1$ & & \\
$\text{LA\_id}_2$ & & \\
. & & \\
. & & \\
. & & \\
$\text{LA\_id}_{\text{NO\_LA}}$ & & \\
\hline
\end{tabular}
\end{center}

The size of each record in $\text{LA\_DB}$ is the sum of the size of $\text{LA\_id}$ attribute, X-Y coordinate attribute, $\text{NO\_Grid}$ attribute. Now the size of $\text{LA\_id}$ attribute is $\log_2(\text{NO\_LA})$ bits, the size of X-Y coordinate attribute is assumed as 16 bits, and the size of $\text{NO\_Grid}$ attribute is $\log_2(\text{NO\_G})$ bits. The number of records in $\text{LA\_DB}$ is equal to the number of LA in the target area. Hence the size of $\text{LA\_DB}$ ($\text{Size\_LA\_DB}$) is $\text{NO\_LA} * (\log_2(\text{NO\_LA}) + 16 + \log_2(\text{NO\_G}))$ bits.

\noindent\textbf{Format and size of Robot\_DB:} Let BS deploys Robot\_1 (R1) in $\text{LA\_id}_1$ and Robot\_2 (R2) in $\text{LA\_id}_{\text{NO\_LA}}$. So the assigned LA\_id ($\text{A\_LA\_id}$) for R1 is $\text{LA\_id}_1$ and for R2 is $\text{LA\_id}_{\text{NO\_LA}}$. BS inserts two records in $\text{Robot\_DB}$ in the form (Robot\_id, $\text{A\_LA\_id}$).

\begin{center}
\begin{tabular}{|l|l|}
\hline
Robot\_id & A\_LA\_id \\
\hline
R1 & $\text{LA\_id}_1$ \\
R2 & $\text{LA\_id}_{\text{NO\_LA}}$ \\
\hline
\end{tabular}
\end{center}

The size of each record in $\text{Robot\_DB}$ is the sum of the size of $\text{Robot\_id}$ attribute and $\text{A\_LA\_id}$ attribute. Now the size of $\text{Robot\_id}$ attribute is 1 bit, its value is 0 for R1 and 1 for R2. The size of $\text{A\_LA\_id}$ attribute is $\log_2(\text{NO\_LA})$ bits. The number of records in $\text{Robot\_DB}$ is 2. Hence the size of $\text{Robot\_DB}$ ($\text{Size\_Robot\_DB}$) is $2 * (1 + \log_2(\text{NO\_LA}))$ bits.

\noindent\textbf{Format and size of Grid\_DB:} $\text{Grid\_Status}$ is the status of a particular grid in a LA. All the grids are considered as uncovered before the deployment of robot and hence the value of $\text{Grid\_Status}$ attribute in all the records of $\text{Grid\_DB}$ is considered as 0.


\begin{center}
\begin{tabular}{|l|l|l|}
\hline
Grid\_id & X-Y coordinate of the centre of grid & Grid\_Status \\
\hline
 & & \\ % Empty row to match the visual style if needed, or remove if content should be there
\hline
\end{tabular}
\end{center}

The size of each record in $\text{Grid\_DB}$ is the sum of the size of $\text{Grid\_id}$ attribute, size of X-Y coordinate attribute and size of $\text{Grid\_Status}$ attribute. Now the size of $\text{Grid\_id}$ attribute is $\log_2(\text{NO\_G})$ bits, the size of X-Y coordinate attribute is assumed as 16 bits, and the size of $\text{Grid\_Status}$ attribute is 1 bit as a grid can be covered or uncovered. The value of $\text{Grid\_Status}$ attribute in a record is 0 for uncovered grid and 1 for covered grid. The number of records in $\text{Grid\_DB}$ is $\text{NO\_G}$. Hence the size of $\text{Grid\_DB}$ ($\text{Size\_Grid\_DB}$) is $\text{NO\_G} * (\log_2(\text{NO\_G}) + 17)$ bits.

\noindent\textbf{Format and size of Sensor\_DB:} $\text{Sensor\_Status}$ is the status of a particular sensor. A sensor can be idle or active and accordingly the value of this attribute is 0 or 1. All the sensors are idle at the time of random deployment and hence initially the value of this attribute in all the records of $\text{Sensor\_DB}$ is 0.

\begin{center}
\begin{tabular}{|l|l|l|}
\hline
Sensor\_id & X-Y coordinate of the sensor & Sensor\_Status \\
\hline
 & & \\ % Empty row to match the visual style if needed, or remove if content should be there
\hline
\end{tabular}
\end{center}

The size of each record in $\text{Sensor\_DB}$ is the sum of the size of $\text{Sensor\_id}$ attribute, size of X-Y coordinate attribute and the size of $\text{Sensor\_Status}$ attribute. Now the size of $\text{Sensor\_id}$ attribute is $\log_2 N_s$ bits, the size of X-Y coordinate attribute is assumed as 16 bits, and the size of $\text{Sensor\_Status}$ attribute is 1 bit. Hence the size of $\text{Sensor\_DB}$ ($\text{Size\_Sensor\_DB}$) is $N_s(\log_2 N_s + 17)$ bits.

\noindent\textbf{x. Global phase}

The global phase is elaborated for $p^{th}$ robot ($\text{Robot\_p}$, $1 \le p \le 2$). Initially let BS deploys $\text{Robot\_p}$ in $q^{th}$ LA ($\text{LA}_q$, $1 \le q \le \text{NO\_LA}$)
Robot\_p executes local phase in $\text{LA}_q$ as discussed in section y. At the end of execution of the local phase Robot\_p counts the number of records ($\text{NO\_Grid}$) in $\text{Grid\_DB}$ in which the $\text{Grid\_Status}$ attribute value is 1 and sends a message ($\text{Robot\_pM}$) to BS in the form (Robot\_p, $\text{NO\_Grid}$)
After receiving $\text{Robot\_pM}$ from $\text{Robot\_p}$ BS
\begin{itemize}
    \item Reads the value of $\text{NO\_Grid}$ from $\text{Robot\_pM}$
    \item Searches $\text{Robot\_DB}$ for $\text{Robot\_p}$ to read the assigned $\text{LA\_id}$ i.e. $\text{LA}_q$
    \item Searches $\text{LA\_DB}$ for $\text{LA}_q$ to update the value of $\text{NO\_Grid}$ attribute from 0 to the value as obtained from $\text{Robot\_pM}$
    \item Searches $\text{LA\_DB}$ to identify a record in which the value of $\text{NO\_Grid}$ attribute is 0
    \item If found deploys $\text{Robot\_p}$ in the LA corresponding to the identified record
\end{itemize}
Otherwise stops robot deployment and computes percentage of area coverage ($\text{Per\_AC}$) as (Number of covered grids/($\text{NO\_G} * \text{NO\_LA}$))*100.

\noindent\textbf{y. Local phase}

The execution of local phase by $\text{Robot\_p}$ in $\text{LA}_q$ is elaborated in this section. The local phase includes the topology discovery phase and dispersion phase. $\text{Robot\_p}$ divides $\text{LA}_q$ in $\text{NO\_G}$ number of grids, inserts $\text{NO\_G}$ number of records in $\text{Grid\_DB}$ and initiates topology discovery phase. The BS is deliberately kept uninformed about such grid formation by $\text{Robot\_p}$ in $\text{LA}_q$. This intentional omission serves to mitigate both communication and storage overheads at BS. By decentralizing the grid formation process and allowing $\text{Robot\_p}$ to autonomously handle this task, the need for continuous communication between $\text{Robot\_p}$ and BS is reduced. However, it is important to ensure that $\text{Robot\_p}$ possesses sufficient computational capabilities to effectively manage the grid formation process autonomously.

\subsubsection*{Topology discovery phase}
Topology discovery is a critical process in WSNs that entails understanding how sensors are spatially arranged within the target area. In this phase, $\text{Robot\_p}$ moves to the centre of $\text{LA}_q$ and initiates the discovery phase from this central position. $\text{Robot\_p}$ broadcasts a message (Mp) in the form ($\text{Robot\_p}$) and receives a reply ($\text{Sensor\_M}$) in the form ($\text{Sensor\_id}$, X-Y coordinate of sensor, $\text{Sensor\_Status}$) from all the sensors within its perception range.
$\text{Robot\_p}$ constructs a spatial map of $\text{LA}_q$ revealing the positions and connectivity of individual sensor nodes within $\text{LA}_q$ and identifies the sensors which are within its perception range from the received replies, marks the $\text{Sensor\_Status}$ of such sensors as idle and inserts $\text{Sensor\_M}$ of the identified sensors in a database ($\text{Sensor\_DB}$). $\text{Robot\_p}$ maintains $\text{Sensor\_DB}$ locally without any communication with BS. $\text{Robot\_p}$ moves within $\text{LA}_q$ to evenly cater the existing sensors.

$\text{Robot\_p}$ starts to executes the dispersion phase after completing the execution of the topology discovery phase and the acquisition of adequate information regarding the surrounding environment, including the number of sensors present in $\text{LA}_q$ and their respective coordinates.

\subsubsection*{Dispersion phase}
It is the process of sensor redeployment by $\text{Robot\_p}$ in the $i^{th}$ grid ($G_i$, $1 \le i \le \text{NO\_G}$) of $\text{LA}_q$. It is elaborated in this section. $\text{Robot\_p}$ reduces $\text{NO\_P}$ by 1 after visiting $G_i$.
There are four possible cases of sensor redeployment by $\text{Robot\_p}$ in $G_i$ of $\text{LA}_q$

\noindent\textbf{Case\_1:} $\text{Robot\_p}$ has sensor(s) in $\text{Stock\_RS}$ and $G_i$ has sensor(s) after random deployment $\text{Robot\_p}$
\begin{itemize}
    \item Places a sensor from $\text{Stock\_RS}$ at the centre of $G_i$ and marks the status of this sensor as active
    \item Inserts a record in $\text{Sensor\_DB}$ in the form ($\text{Sensor\_id}$, X-Y coordinate, $\text{Sensor\_Status}$) where the $\text{Sensor\_Status}$ value for this sensor is 1
    \item Collects all the extra sensors from $G_i$ till $\text{Stock\_RS}$ is less than 15
    \item Removes the records corresponding to the collected sensors from $\text{Sensor\_DB}$
    \item Considers $G_i$ as covered
    \item Searches $\text{Grid\_DB}$ for $G_i$ and updates $\text{Grid\_Status}$ value from 0 to 1
\end{itemize}

\noindent\textbf{Case\_2:} $\text{Robot\_p}$ has sensors in $\text{Stock\_RS}$ but $G_i$ has no sensors after random deployment $\text{Robot\_p}$
\begin{itemize}
    \item Places a sensor from $\text{Stock\_RS}$ at the centre of $G_i$ and marks the status of this sensor as active
    \item Inserts a record in $\text{Sensor\_DB}$ in the form ($\text{Sensor\_id}$, X-Y coordinate, $\text{Sensor\_Status}$) where the $\text{Sensor\_Status}$ value for this sensor is 1
    \item Searches $\text{Grid\_DB}$ for $G_i$ and updates $\text{Grid\_Status}$ value from 0 to 1
\end{itemize}

\noindent\textbf{Case\_3:} $\text{Robot\_p}$ has no sensors in $\text{Stock\_RS}$ but $G_i$ has sensors after random deployment $\text{Robot\_p}$
\begin{itemize}
    \item Calculates the distance between the centre of $G_i$ and the location of sensors in $G_i$ as obtained from $\text{Sensor\_DB}$
    \item Identifies a sensor for which the calculated distance is minimum to keep the robot movement within limit and places that sensor at the grid centre
    \item Searches $\text{Sensor\_DB}$ for the record of the identified sensor
    \item Updates this record by replacing the existing X-Y coordinate by the X-Y coordinate of the centre of $G_i$ and by changing the $\text{Sensor\_Status}$ of this sensor from 0 to 1
    \item Collects all the extra sensors from $G_i$ till $\text{Stock\_RS}$ is less than 15
\end{itemize}


\begin{itemize}
    \item Removes the records corresponding to the collected sensors from $\text{Sensor\_DB}$
    \item Considers $G_i$ as covered
    \item Searches $\text{Grid\_DB}$ for $G_i$ and updates $\text{Grid\_Status}$ value from 0 to 1
\end{itemize}

\noindent\textbf{Case\_4:} $\text{Robot\_p}$ has no sensors in $\text{Stock\_RS}$ and $G_i$ has no sensor after random deployment
In this case $G_i$ remains uncovered.

$\text{Robot\_p}$ finds the nearest uncovered grid from $G_i$ within $\text{LA}_q$ by searching $\text{Grid\_DB}$ and repeats the dispersion phase for the new grid. $\text{Robot\_p}$ iteratively executes the dispersion phase for $\text{NO\_G}$ grids within $\text{LA}_q$ to conclude the current local phase. By limiting the number of moves to match the number of grids, $\text{Robot\_p}$ optimizes its movement, ensuring efficient utilization of its resources while working towards the goal of complete coverage. This constraint serves to regulate the movement behaviour of $\text{Robot\_p}$, ensuring efficient resource utilization and facilitating systematic sensor redeployment within $\text{LA}_q$. Upon completion $\text{Robot\_p}$ resets $\text{NO\_P}$ to its initial value, $\text{NO\_G}$. It then searches $\text{Grid\_DB}$ for records where the $\text{Grid\_Status}$ attribute equals to 1, counts the number of such records ($\text{Cov\_G}$) to find the number of covered grids in $\text{LA}_q$ and sends a message ($\text{Robot\_pM}$) to BS in the form ($\text{Robot\_p}$, $\text{Cov\_G}$)

Through this centralized update mechanism, BS gains valuable insight into the coverage status of the LAs, enabling informed decision making and coordination of network-wise activities.

\subsubsection*{u. Pseudo code}

In this section, the pseudocode for APP\_I is discussed and the flowchart in Figure 5.2 demonstrates its step-by-step execution.

% The actual flowchart image is not provided in the text, so I'm commenting out the \includegraphics
% If you have the image, you can place it here.
% \begin{figure}[h!]
%     \centering
%     % \includegraphics[width=0.8\textwidth]{your_flowchart_image_filename} % Replace with your image file
%     \caption{The Flowchart of APP\_I}
%     \label{fig:app_i_flowchart}
% \end{figure}
\captionsetup{labelformat=empty,textformat=empty} % To prevent "Figure X:" prefix if only caption text is needed
\captionof{figure}{\textbf{Figure 5.2: The Flowchart of APP\_I}}
\captionsetup{labelformat=default,textformat=default} % Restore default caption formatting

Algorithm 1 orchestrates the entire APP\_I for sensor deployment. It begins by defining the target area and partitioning it into local areas (LAs) based on the robot’s perception range (R). $\text{Robot\_DB}$, $\text{LA\_DB}$, $\text{Grid\_DB}$, and $\text{Sensor\_DB}$ are initialized. This phase continues as long as there are records in the $\text{LA\_DB}$ with an attribute value of NO Grid equal to 0. For each Robot p from 1 to 2, the BS searches the $\text{LA\_DB}$ to find a local area $\text{LA}_q$ with NO Grid attribute equal to 0 and assigns it to the robot. The Robot DB is updated with this information.

\clearpage % To put algorithms on a new page as suggested by layout

\noindent\textbf{Algorithm 1}
\begin{enumerate}[label=\arabic*:]
    \item Initialization:
    \item Define target area and partition into local areas (LAs) based on robot perception range (R).
    \item Initialize databases: $\text{Robot\_DB}$, $\text{LA\_DB}$, $\text{Grid\_DB}$, $\text{Sensor\_DB}$.
\end{enumerate}

\begin{enumerate}[label=\arabic*:, start=4]
    \item Global Phase:
    \item while there exists a record in $\text{LA\_DB}$ whose $\text{NO\_Grid}$ attribute value is 0 do
    \item \quad for Robot p $\leftarrow$ 1 to 2 do
    \item \quad \quad BS searches the $\text{LA\_DB}$ to find a $\text{LA}_q$ with $\text{NO\_Grid}$ attribute = 0
    \item \quad \quad BS assigns the $\text{LA}_q$ to $\text{Robot\_p}$.
    \item \quad \quad Update $\text{Robot\_DB}$.
    \item \quad \quad Local Phase
    \item \quad \quad BS receives the message $\text{Robot\_pM}$
    \item \quad \quad Updates the corresponding $\text{LA}_q$ record in $\text{LA\_DB}$.
    \item \quad end for
    \item end while
    \item Compute $\text{Per\_AC}$
\end{enumerate}

Algorithm 2 divides the assigned $\text{LA}_q$ into a specified number of grids ($\text{NO\_G}$) and updates the $\text{Grid\_DB}$. Topology Discovery is performed, followed by initialization of $\text{NO\_P}$ to $\text{NO\_G}$. After Dispersion Phase is executed, $\text{Robot\_p}$ counts the total number of grids covered and sends a message $\text{Robot\_pM}$ to the BS.

\noindent\textbf{Algorithm 2 Local phase}
\begin{enumerate}[label=\arabic*:]
    \item Divide $\text{LA}_q$ into $\text{NO\_G}$ grids.
    \item Insert records into $\text{Grid\_DB}$
    \item Topology Discovery Phase
    \item Initialize $\text{NO\_P}$ = $\text{NO\_G}$ (number of permissible moves).
    \item Dispersion Phase
    \item Search $\text{Grid\_DB}$ for the record with $\text{Grid\_Status}$ attribute value equal to 1
    \item Count covered grids $\text{COV\_G}$ in $\text{LA}_q$ from $\text{Grid\_DB}$.
    \item Send message $\text{Robot\_pM}$ to BS in format ($\text{Robot\_p}$, $\text{COV\_G}$).
\end{enumerate}

Algorithm 3 the robot moves to the centre of $\text{LA}_q$, which is also the centre of a specific grid $G_i$. Then the robot broadcasts a message (Mp) to discover sensors within its perception range, receiving responses ($\text{Sensor\_M}$) from the sensors, and updating the Sensor DB accordingly.

\noindent\textbf{Algorithm 3 Topology Discovery Phase}
\begin{enumerate}[label=\arabic*:]
    \item Move $\text{Robot\_p}$ to centre of $\text{LA}_q$, i.e., also the centre of Grid $G_i$
    \item Broadcast message (Mp) to discover sensors.
    \item Receive responses ($\text{Sensor\_M}$) from sensors within $\text{Robot\_p}$’s perception range.
    \item Update $\text{Sensor\_DB}$ with $\text{Sensor\_M}$ information.
\end{enumerate}

Algorithm 4 runs the dispersion of sensors in each individual local area $\text{LA}_q$ until $\text{NO\_P}$ is reduced to 0. The dispersion of each sensor depends on the four cases discussed earlier in section y, followed by a decrease in $\text{NO\_P}$. Then moving on to the next nearest uncovered grid.

\noindent\textbf{Algorithm 4 Dispersion Phase}

\noindent\texttt{1: \quad while NO\_P > 0 do} \\
\noindent\texttt{2: \quad \quad Case 1: If Stock\_RS has sensors and G$_i$ has sensors:} \\
\noindent\texttt{3: \quad \quad \quad Place a sensor from Stock\_RS at the centre of G$_i$} \\
\noindent\texttt{4: \quad \quad \quad Insert a record in Sensor\_DB.} \\
\noindent\texttt{5: \quad \quad \quad Collect extra sensors from G$_i$ until Stock\_RS capacity is reached.} \\
\noindent\texttt{6: \quad \quad \quad Remove records of collected sensors from Sensor\_DB.} \\
\noindent\texttt{7: \quad \quad \quad Mark G$_i$ as covered in Grid\_DB.} \\
\noindent\texttt{8: \quad \quad Case 2: If Stock\_RS has sensors but G$_i$ has no sensors:} \\
\noindent\texttt{9: \quad \quad \quad Place a sensor from Stock\_RS at the centre of G$_i$} \\
\noindent\texttt{10:\quad \quad \quad Insert a record in Sensor\_DB.} \\
\noindent\texttt{11:\quad \quad \quad Mark G$_i$ as covered in Grid\_DB.} \\
\noindent\texttt{12:\quad \quad Case 3: If Stock\_RS has no sensors but G$_i$ has sensors :} \\
\noindent\texttt{13:\quad \quad \quad Collect extra sensors from G$_i$ until Stock\_RS capacity is reached.} \\
\noindent\texttt{14:\quad \quad \quad Remove records of collected sensors from Sensor\_DB.} \\
\noindent\texttt{15:\quad \quad \quad Place a sensor from Stock\_RS at the center of G$_i$} \\
\noindent\texttt{16:\quad \quad \quad Mark G$_i$ as covered in Grid\_DB.} \\
\noindent\texttt{17:\quad \quad Case 4: If neither Stock\_RS nor G$_i$ has sensors :} \\
\noindent\texttt{18:\quad \quad \quad Leave G$_i$ uncovered.} \\
\noindent\texttt{19:\quad \quad Decrement NO\_P by 1.} \\
\noindent\texttt{20:\quad \quad Find the nearest uncovered Grid G$_i$} \\
\noindent\texttt{21:\quad end while}

\subsubsection*{Format and size of messages}

\noindent\textbf{Format and size of Robot\_pM:} Robot\_p sends $\text{Robot\_pM}$ in the form ($\text{Robot\_p}$, $\text{Cov\_G}$) to BS at the end of its current local phase. The size of $\text{Robot\_p}$ is 1 bit and $\text{Cov\_G}$ is $\log_2(\text{NO\_G})$ bits. Hence the size of $\text{Robot\_pM}$ ($\text{Size\_Robot\_pM}$) is $1 + \log_2(\text{NO\_G})$ bits.

\noindent\textbf{Format and size of Mp:} Robot\_p broadcasts Mp in the form ($\text{Robot\_p}$) after its deployment in LA$_q$. The size of Mp ($\text{Size\_Mp}$) is 1 bit.

\noindent\textbf{Format and size of Sensor\_M:} Each sensor in the perception range of $\text{Robot\_p}$ sends $\text{Sensor\_M}$ in the form ($\text{Sensor\_id}$, X-Y coordinate of the sensor, $\text{Sensor\_Status}$) as reply of Mp. The size of $\text{Sensor\_M}$ ($\text{Size\_Sensor\_M}$) is $(\log_2 N_s + 17)$ bits.

\subsection*{v. Energy Consumption}

This section discusses the energy consumption model of APP\_I. The model focuses on quantifying the energy consumption of sensor nodes, robots, and the base station, considering various operational modes and activities. The total energy consumption ($\text{Energy}_{\text{tot}}$) in APP\_I is the summation of the energy consumption at the BS, robots, and sensor nodes. By accurately estimating energy consumption, this model facilitates the optimization of efficient deployment strategies in WSNs. In WSN, energy constraints pose significant challenges to deployment, as sensor nodes, robots, and base stations must operate efficiently to prolong the network lifetime.

\subsubsection*{Energy Consumption in Sensor Nodes}

Sensor nodes operate in two distinct modes: Active Mode and Idle Mode, each characterized by specific components of energy consumption.

\noindent\textbf{Active Mode:} In this mode, the energy consumption ($E_{\text{active}}$) of a sensor node consists of baseline energy, processing energy, and radio energy.
\begin{itemize}
    \item \textbf{Baseline Energy ($E_{\text{baseline}}$):}
\end{itemize}
The baseline energy consumption is determined by the time duration ($t_i - t_s$) and the constant power consumption $P_{\text{baseline}}$ during this period. The time $t_i$ represents the interval associated with the operational cycle, calculated as
\[
t_i = I_C \times (T_C + T_P), \text{ where } I_C \text{ is the interval count, } T_C \text{ is the cycle time, and } T_P \text{ is the processing time.}
\]
\[
E_{\text{baseline}} = (t_i - t_s) \times P_{\text{baseline}}
\]
\begin{itemize}
    \item \textbf{Sensing Energy ($E_{\text{sensing}}$):}
\end{itemize}
The energy consumed for the radiation task, which is in accordance with the sensing range of a sensor [25].
\[
E_{\text{sensing}} = \mu r_i^2
\]
where $\mu$ is the energy consumed by a sensor node for the sensing field and the value of $\mu$ is 0.0005, and $r_i$ represents the sensing range of the $i^{th}$ sensor.
\begin{itemize}
    \item \textbf{Processing Energy ($E_{\text{processing}}$):}
\end{itemize}
Energy consumed during processing tasks is given by
\[
E_{\text{processing}} = P_{\text{processing}} \times t_{\text{processing}}
\]
where $P_{\text{processing}}$ denotes the power consumed during processing and $t_{\text{processing}}$ represents the processing time.
\begin{itemize}
    \item \textbf{Radio Energy ($E_{\text{radio}}$):}
\end{itemize}
Radio energy consumption includes the energy consumed for transmission, receiving and switching operations.
\[
E_{\text{radio}} = E_{\text{transmit}} + E_{\text{receive}} + E_{\text{switch}}
\]
\begin{itemize}
    \item[\quad --] \textbf{Transmit Energy ($E_{\text{transmit}}$):} Energy used for transmitting data packets
    \[
    E_{\text{transmit}} = P_{\text{transmit}} \times t_{\text{transmit}}
    \]
    where $P_{\text{transmit}}$ denotes the power consumed during transmission and $t_{\text{transmit}}$ represents the transmission time.
    \item[\quad --] \textbf{Receive Energy ($E_{\text{receive}}$):} Energy used for receiving data packets
    \[
    E_{\text{receive}} = P_{\text{receive}} \times t_{\text{receive}}
    \]
    where $P_{\text{receive}}$ denotes the power consumed during reception and $t_{\text{receive}}$ represents the reception time.
\end{itemize}
The total energy consumption in Active Mode is given by the sum of baseline, sensing, processing, and radio energy.
\[
E_{\text{active}} = E_{\text{baseline}} + E_{\text{sensing}} + E_{\text{processing}} + E_{\text{radio}}
\]

\noindent\textbf{Idle Mode:} In this mode, sensor nodes are inactive in sensing their surroundings, resulting in reduced energy consumption. The energy consumed in idle mode ($E_{\text{idle}}$) is the summation of baseline and radio energy.

\[
E_{\text{idle}} = E_{\text{baseline}} + E_{\text{radio}}
\]

\subsubsection*{Energy Consumption in Robot}

Robots contribute to energy consumption through baseline operations, radio communications, and mobility.
\begin{itemize}
    \item \textbf{Baseline Energy ($E_{\text{baseline\_robot}}$):}
\end{itemize}
Baseline energy consumption by robots is calculated in a similar way as discussed for sensor nodes.
\[
E_{\text{baseline\_robot}} = (t_i - t_s) \times P_{\text{baseline\_robot}}
\]
\begin{itemize}
    \item \textbf{Radio Energy ($E_{\text{radio\_robot}}$):}
\end{itemize}
Radio energy consumption in robots includes transmit, receive and switch energies, is the same as discussed for sensor nodes
\begin{itemize}
    \item \textbf{Mobility Energy ($E_{\text{mobility}}$):}
\end{itemize}
During sensor redeployment, each robot requires energy for the task of moving the sensors. Energy consumed by robots for movement is calculated as the product of distance moved and energy per unit distance [26].
\[
E_{\text{mobility}} = \tau D_p
\]
where $D_p$ is the distance covered by the $p^{th}$ robot, and $\tau$ is the coefficient of the energy consumed during robot movement. $\tau = 0.0005$ is considered according to [26].

The total energy consumption in robot ($E_{\text{robot}}$) is the sum of baseline, radio, and mobility energy.
\[
E_{\text{robot}} = E_{\text{baseline\_robot}} + E_{\text{radio\_robot}} + E_{\text{mobility}}
\]

\subsubsection*{Energy Consumption in Base Station}

The base station consumes energy for processing data and radio communications.
\begin{itemize}
    \item \textbf{Processing Energy ($E_{\text{processing\_base}}$):}
\end{itemize}
Energy consumed for processing tasks at the base station, is the same as discussed for sensor.
\begin{itemize}
    \item \textbf{Radio Energy ($E_{\text{radio\_base}}$):}
\end{itemize}
Energy consumed for radio communications, including transmit and receive operations, is the same as discussed for sensor.

The total energy consumption at the base station ($E_{\text{base\_station}}$) is the sum of processing and radio energy.
\[
E_{\text{base\_station}} = E_{\text{processing\_base}} + E_{\text{radio\_base}}
\]

So the total energy consumption for APP\_I is
\[
\mathbf{Energy_{\text{Tot}}} = \mathbf{E_{\text{active}}} + \mathbf{E_{\text{idle}}} + \mathbf{E_{\text{robot}}} + \mathbf{E_{\text{base\_station}}}
\]

\end{document}