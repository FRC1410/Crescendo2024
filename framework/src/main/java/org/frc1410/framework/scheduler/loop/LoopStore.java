package org.frc1410.framework.scheduler.loop;

import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.frc1410.framework.phase.Phase;
import org.frc1410.framework.scheduler.task.TaskScheduler;
import org.frc1410.framework.util.log.Logger;
import org.jetbrains.annotations.NotNull;

import java.util.*;

public final class LoopStore {

	private static final Logger LOG = new Logger("LoopStore");

	private final TaskScheduler scheduler;
	private final Long2ObjectOpenHashMap<Loop> loops = new Long2ObjectOpenHashMap<>();
	private final Deque<Loop> untracked = new ArrayDeque<>();
	public final Loop main;

	public LoopStore(TaskScheduler scheduler) {
		this.scheduler = scheduler;
		this.main = new Loop(scheduler, -1L);
	}

	public Loop ofPeriod(long period) {
		Loop loop = loops.get(period);
		if (loop == null) {
			loop = new Loop(scheduler, period);
			loops.put(period, loop);
			untracked.add(loop);

			LOG.info("Registered new loop #%08x running at period of %dms", loop.hashCode(), period);
		}

		return loop;
	}

	public void propagateTransition(@NotNull Phase newPhase) {
		main.flagTransition(newPhase);
		
		for (var loop : loops.values()) {
			loop.flagTransition(newPhase);
		}
	}

	public Collection<Loop> getLoops(boolean includeMain) {
		if (includeMain) {
			var set = new HashSet<>(loops.values());
			set.add(main);
			return set;
		}

		return loops.values();
	}

	public Deque<Loop> getUntrackedLoops() {
		return untracked;
	}
}
