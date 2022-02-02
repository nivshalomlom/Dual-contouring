using System.Collections.Generic;
using System.Threading;
using System;

/// <summary>
/// Thread pool implementation in C#
/// </summary>
public class ThreadPool
{

    // Constants
    private const int THREAD_IDLE_TIMEOUT = 1000;
    
    // Task pool
    private Queue<Action> taskPool;
    private object poolLock;

    // Thread events
    private ManualResetEvent terminateEvent;
    private ManualResetEvent stopEvent;

    // Threads
    private Thread[] threads;
    private bool[] idleFlags;

    /// <summary>
    /// A constructor to build a new thread pool
    /// </summary>
    /// <param name="numThreads"> The nubmer of the threads to build in the pool </param>
    public ThreadPool(int numThreads)
    {
        // Initialize pool
        this.taskPool = new Queue<Action>();
        this.poolLock = new object();

        // Initialize threads events
        this.terminateEvent = new ManualResetEvent(false);
        this.stopEvent = new ManualResetEvent(false);

        // Initialize threads
        this.threads = new Thread[numThreads];
        this.idleFlags = new bool[numThreads];

        // Build all threads
        for (int i = 0; i < numThreads; i++)
        {
            // Reset flags and save id
            this.idleFlags[i] = false;
            int tid = i;

            this.threads[i] = new Thread(() => 
            {
                // Start work loop
                while (true)
                {
                    // If stopped wait for signal
                    this.stopEvent.WaitOne(Timeout.Infinite);

                    // Exit thread if terminate event triggered
                    if (this.terminateEvent.WaitOne(0))
                        break;

                    // Try and get a task from the pool
                    Action task = null;
                    lock (this.poolLock)
                    {
                        if (this.taskPool.Count == 0)
                            this.idleFlags[tid] = true;
                        else
                        {
                            this.idleFlags[tid] = false;
                            task = this.taskPool.Dequeue();
                        }
                    }

                    // If idle timeout
                    if (this.idleFlags[tid])
                        Thread.Sleep(THREAD_IDLE_TIMEOUT);
                    else task();
                }
            });
            this.threads[i].Start();
        }
    }

    // A destructor to safely terminate all pool threads before object destruction
    ~ThreadPool()
    {
        // Tell all threads to terminate
        this.stopEvent.Set();
        this.terminateEvent.Set();

        // For each thread wait for exit
        for (int i = 0; i < this.threads.Length; i++)
            this.threads[i].Join();
    }

    /// <summary>
    /// A method to add a task to the pool
    /// </summary>
    /// <param name="action"> A lambda expression of the task to be done </param>
    /// <returns> A manual reset event the will be signaled when this task is finished executing </returns>
    public ManualResetEvent AddTask(Action action)
    {
        ManualResetEvent completionEvent = new ManualResetEvent(false);
        lock (this.poolLock)
            this.taskPool.Enqueue(() => 
            {
                action();
                completionEvent.Set();
            });
        return completionEvent;
    }

    /// <returns> True if all threads are idle, false otherwise </returns>
    public bool AllThreadsIdle()
    {
        foreach (bool flag in this.idleFlags)
            if (!flag)
                return false;
        return true;
    }

    /// <summary>
    /// A method to start all the threads in the pool
    /// </summary>
    public void Start() => this.stopEvent.Set();

    /// <summary>
    /// A method to stop all the threads in the pool
    /// </summary>
    public void Stop() => this.stopEvent.Reset();

    /// <returns> True if no tasks left in the pool, false otherwise </returns>
    public bool IsEmpty() => this.taskPool.Count == 0;

}
