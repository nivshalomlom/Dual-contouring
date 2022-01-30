using System.Threading;
using System;

public class ThreadPool
{

    private MaxHeap<PoolTask> taskPool;
    private object poolLock;

    private PoolThread[] threads;

    public ThreadPool(int numberOfThreads)
    {
        this.taskPool = new MaxHeap<PoolTask>();
        this.poolLock = new object();

        this.RebuildThreads(numberOfThreads);
    }

    ~ThreadPool() => this.Terminate();

    public void AddTask(Action task, int priority)
    {
        lock (this.poolLock)
            this.taskPool.Insert(new PoolTask(task, priority));
    }

    public void ClearPool()
    {
        lock (this.poolLock)
            this.taskPool.Clear();
    }

    public void Start() 
    {
        if (this.threads == null)
            throw new Exception("Error!: to reuse a terminated thread pool call RebuildThreads!");

        foreach (PoolThread pt in this.threads)
            pt.Start();
    }

    public void Stop() 
    {
        if (this.threads == null)
            throw new Exception("Error!: to reuse a terminated thread pool call RebuildThreads!");

        foreach (PoolThread pt in this.threads)
            pt.Stop();
    }

    public void Terminate()
    {
        foreach (PoolThread pt in this.threads)
            pt.Terminate();
        
        this.threads = null;
    }

    public void RebuildThreads(int numberOfThreads)
    {
        this.threads = new PoolThread[numberOfThreads];

        for (int i = 0; i < numberOfThreads; i++)
            this.threads[i] = new PoolThread(this);
    }

    public bool isEmpty() => this.taskPool.IsEmpty();

    public bool allThreadIdle()
    {
        if (this.threads == null)
            throw new Exception("Error!: to reuse a terminated thread pool call RebuildThreads!");

        bool result = true;
        foreach (PoolThread pt in this.threads)
            result &= pt.IsIdle() || !pt.IsRunning();

        return result;
    }

    #region Data structures

    /// <summary>
    /// A struct to define a task in the pool
    /// </summary>
    struct PoolTask : IComparable<PoolTask>
    {
        private Action task;
        public int priority;

        /// <summary>
        /// A constructor to create a new pool task
        /// </summary>
        /// <param name="task"> A lambda expression to be executed </param>
        /// <param name="priority"> The priority of this task, the higher the priority the sooner this task will be executed </param>
        public PoolTask(Action task, int priority)
        {
            this.task = task;
            this.priority = priority;
        }

        /// <summary>
        /// A method to compare to tasks based on their priority
        /// </summary>
        /// <param name="other"> The task to compare to </param>
        /// <returns> -1 if other task is more urgent, 1 if this task more urgent, 0 if the same priority </returns>
        public int CompareTo(PoolTask other) => this.priority.CompareTo(other.priority);

        /// <summary>
        /// A method to run the task
        /// </summary>
        public void Run() => this.task();

    }

    /// <summary>
    /// A class to represent a thread in the pool
    /// </summary>
    class PoolThread
    {
        private static int IdleCooldown = 1000;

        private Thread thread;
        private ThreadPool container;
        private bool isIdle, isHalted, isAlive;

        /// <summary>
        /// A constructor to create a new pool thread
        /// </summary>
        /// <param name="container"> The thread pool containing this thread </param>
        public PoolThread(ThreadPool container)
        {
            // Intiallize link to pool and internal flags
            this.container = container;
            this.isIdle = false;
            this.isHalted = true;
            this.isAlive = true;

            // Create and the start the thread
            this.thread = new Thread(() =>
            {
                while (this.isAlive)
                {
                     // If thread is stopped wait for start order
                    if (this.isHalted)
                    {
                        Thread.Sleep(IdleCooldown);
                        continue;
                    }
                    
                    // Try and get a task from the pool
                    PoolTask task = default(PoolTask);
                    lock (container.poolLock)
                    {
                        // If pool is empty go into idle mode
                        if (container.taskPool.IsEmpty())
                            this.isIdle = true;
                        else 
                        {
                            task = container.taskPool.ExtractMax();
                            this.isIdle = false;
                        }
                    }

                    // If idle enter a cooldown
                    if (this.isIdle)
                        Thread.Sleep(IdleCooldown);
                    else task.Run();
                }
            });
            this.thread.Start();
        }

        /// <summary>
        /// A destructor to make sure the threads stops when object is destroyed
        /// </summary>
        ~PoolThread() => this.Terminate();

        /// <returns> Returns true if this thread is currently idle </returns>
        public bool IsIdle() => this.isIdle;

        /// <returns> Returns true if this thread is alive </returns>
        public bool IsRunning() => !this.isHalted;

        /// <summary>
        /// A method to start this thread
        /// </summary>
        public void Start() => this.isHalted = false;

        /// <summary>
        /// A method to stop this thread
        /// </summary>
        public void Stop() => this.isHalted = true;

        /// <summary>
        /// A method to terminate this thread <br/>
        /// Note: after termination this thread cant be reused!
        /// </summary>
        public void Terminate() => this.isAlive = false;

    }

    #endregion

}
