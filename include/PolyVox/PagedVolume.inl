/*******************************************************************************
* The MIT License (MIT)
*
* Copyright (c) 2015 David Williams and Matthew Williams
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*******************************************************************************/

#include "Impl/ErrorHandling.h"

#include <algorithm>
#include <limits>
#include <thread> //NOTE(dynworld): std::this_thread::yield for the paging-flag wait (P3a')

namespace PolyVox
{
	template <typename ResourceType>
	AutoReadLock<ResourceType>::AutoReadLock(const ResourceType* resourceToLock)
		: m_resource(resourceToLock)
	{
		m_resource->readLock();
	}

	template <typename ResourceType>
	AutoReadLock<ResourceType>::~AutoReadLock()
	{
		m_resource->readUnlock();
	}

	template <typename ResourceType>
	AutoWriteLock<ResourceType>::AutoWriteLock(const ResourceType* resourceToLock)
		: m_resource(resourceToLock)
	{
		m_resource->writeLock();
	}

	template <typename ResourceType>
	AutoWriteLock<ResourceType>::~AutoWriteLock()
	{
		m_resource->writeUnlock();
	}

	template <typename AutoReadLockType>
	AutoUpgradeLock<AutoReadLockType>::AutoUpgradeLock(AutoReadLockType& readLock)
		: m_readLock(readLock)
	{
		m_readLock.m_resource->upgradeLock();
	}

	template <typename AutoReadLockType>
	AutoUpgradeLock<AutoReadLockType>::~AutoUpgradeLock()
	{
		m_readLock.m_resource->downgradeLock();
	}

	////////////////////////////////////////////////////////////////////////////////
	/// This constructor creates a volume with a fixed size which is specified as a parameter. By default this constructor will not enable paging but you can override this if desired. If you do wish to enable paging then you are required to provide the call back function (see the other PagedVolume constructor).
	/// \param pPager Called by PolyVox to load and unload data on demand.
	/// \param uTargetMemoryUsageInBytes The upper limit to how much memory this PagedVolume should aim to use.
	/// \param uChunkSideLength The size of the chunks making up the volume. Small chunks will compress/decompress faster, but there will also be more of them meaning voxel access could be slower.
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	PagedVolume<VoxelType, RWLockType>::PagedVolume(Pager* pPager, uint32_t uTargetMemoryUsageInBytes, uint16_t uChunkSideLength, IChunkCache* pChunkCache)
		: BaseVolume<VoxelType>()
		, m_pChunkCache(pChunkCache)
		, m_uChunkSideLength(uChunkSideLength)
		, m_uChunkSideLengthPower(logBase2(uChunkSideLength)) // Used to perform multiplications and divisions by bit shifting.
		, m_iChunkMask(uChunkSideLength - 1) // Use to perform modulo by bit operations
		, m_pPager(pPager)
	{
		// Validation of parameters
		POLYVOX_THROW_IF(!pPager, std::invalid_argument, "You must provide a valid pager when constructing a PagedVolume");
		POLYVOX_THROW_IF(!pChunkCache, std::invalid_argument, "You must provide a valid chunk cache handler when constructing a PagedVolume");
		POLYVOX_THROW_IF(uTargetMemoryUsageInBytes < 1 * 1024 * 1024, std::invalid_argument, "Target memory usage is too small to be practical");
		POLYVOX_THROW_IF(m_uChunkSideLength == 0, std::invalid_argument, "Chunk side length cannot be zero.");
		POLYVOX_THROW_IF(m_uChunkSideLength > 256, std::invalid_argument, "Chunk size is too large to be practical.");
		POLYVOX_THROW_IF(!isPowerOf2(m_uChunkSideLength), std::invalid_argument, "Chunk side length must be a power of two.");


		// Calculate the number of chunks based on the memory limit and the size of each chunk.
		uint32_t uChunkSizeInBytes = PagedVolume<VoxelType, RWLockType>::Chunk::calculateSizeInBytes(m_uChunkSideLength);
		m_uChunkCountLimit = uTargetMemoryUsageInBytes / uChunkSizeInBytes;

		// Enforce sensible limits on the number of chunks.
		const uint32_t uMinPracticalNoOfChunks = 32; // Enough to make sure a chunks and it's neighbours can be loaded, with a few to spare.
		const uint32_t uMaxPracticalNoOfChunks = uChunkArraySize / 2; // A hash table should only become half-full to avoid too many clashes.
		POLYVOX_LOG_WARNING_IF(m_uChunkCountLimit < uMinPracticalNoOfChunks, "Requested memory usage limit of ",
			uTargetMemoryUsageInBytes / (1024 * 1024), "Mb is too low and cannot be adhered to.");
		m_uChunkCountLimit = (std::max)(m_uChunkCountLimit, uMinPracticalNoOfChunks);
		m_uChunkCountLimit = (std::min)(m_uChunkCountLimit, uMaxPracticalNoOfChunks);

		// Inform the user about the chosen memory configuration.
		POLYVOX_LOG_DEBUG("Memory usage limit for volume now set to ", (m_uChunkCountLimit * uChunkSizeInBytes) / (1024 * 1024),
			"Mb (", m_uChunkCountLimit, " chunks of ", uChunkSizeInBytes / 1024, "Kb each).");
	}

	////////////////////////////////////////////////////////////////////////////////
	/// Destroys the volume The destructor will call flushAll() to ensure that a paging volume has the chance to save it's data via the dataOverflowHandler() if desired.
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	PagedVolume<VoxelType, RWLockType>::~PagedVolume()
	{
		flushAll();
	}

	////////////////////////////////////////////////////////////////////////////////
	/// This version of the function is provided so that the wrap mode does not need
	/// to be specified as a template parameter, as it may be confusing to some users.
	/// \param uXPos The \c x position of the voxel
	/// \param uYPos The \c y position of the voxel
	/// \param uZPos The \c z position of the voxel
	/// \return The voxel value
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	VoxelType PagedVolume<VoxelType, RWLockType>::getVoxel(int32_t uXPos, int32_t uYPos, int32_t uZPos) const
	{
		AutoReadLock<_Myt> volumeLock(this);

		const int32_t chunkX = uXPos >> m_uChunkSideLengthPower;
		const int32_t chunkY = uYPos >> m_uChunkSideLengthPower;
		const int32_t chunkZ = uZPos >> m_uChunkSideLengthPower;

		const uint16_t xOffset = static_cast<uint16_t>(uXPos & m_iChunkMask);
		const uint16_t yOffset = static_cast<uint16_t>(uYPos & m_iChunkMask);
		const uint16_t zOffset = static_cast<uint16_t>(uZPos & m_iChunkMask);

		std::shared_ptr<Chunk> pChunk = m_pChunkCache->getChunkCached(this, chunkX, chunkY, chunkZ).lock();

		// NOTE(dynworld) P3a': a cached chunk can still be mid-page-in; fall
		// through to getChunk, which waits on the paging flag with no locks held.
		if (pChunk == nullptr || pChunk->isPagingInProgress())
			pChunk = getChunk(chunkX, chunkY, chunkZ);

		AutoReadLock<_Myt::Chunk> chunkLock(pChunk.get());
		return pChunk->getVoxel(xOffset, yOffset, zOffset);
	}

	////////////////////////////////////////////////////////////////////////////////
	/// This version of the function is provided so that the wrap mode does not need
	/// to be specified as a template parameter, as it may be confusing to some users.
	/// \param v3dPos The 3D position of the voxel
	/// \return The voxel value
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	VoxelType PagedVolume<VoxelType, RWLockType>::getVoxel(const Vector3DInt32& v3dPos) const
	{
		return getVoxel(v3dPos.getX(), v3dPos.getY(), v3dPos.getZ());
	}

	////////////////////////////////////////////////////////////////////////////////
	/// \param uXPos the \c x position of the voxel
	/// \param uYPos the \c y position of the voxel
	/// \param uZPos the \c z position of the voxel
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::setVoxel(int32_t uXPos, int32_t uYPos, int32_t uZPos, VoxelType tValue)
	{
		const int32_t chunkX = uXPos >> m_uChunkSideLengthPower;
		const int32_t chunkY = uYPos >> m_uChunkSideLengthPower;
		const int32_t chunkZ = uZPos >> m_uChunkSideLengthPower;

		const uint16_t xOffset = static_cast<uint16_t>(uXPos - (chunkX << m_uChunkSideLengthPower));
		const uint16_t yOffset = static_cast<uint16_t>(uYPos - (chunkY << m_uChunkSideLengthPower));
		const uint16_t zOffset = static_cast<uint16_t>(uZPos - (chunkZ << m_uChunkSideLengthPower));

		std::shared_ptr<Chunk> pChunk = m_pChunkCache->getChunkCached(this, chunkX, chunkY, chunkZ).lock();

		// NOTE(dynworld) P3a': a cached chunk can still be mid-page-in; fall
		// through to getChunk, which waits on the paging flag with no locks held.
		if (pChunk == nullptr || pChunk->isPagingInProgress())
			pChunk = getChunk(chunkX, chunkY, chunkZ);

		AutoWriteLock<_Myt::Chunk> lock(pChunk.get());
		pChunk->setVoxel(xOffset, yOffset, zOffset, tValue);
	}

	////////////////////////////////////////////////////////////////////////////////
	/// \param v3dPos the 3D position of the voxel
	/// \param tValue the value to which the voxel will be set
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::setVoxel(const Vector3DInt32& v3dPos, VoxelType tValue)
	{
		setVoxel(v3dPos.getX(), v3dPos.getY(), v3dPos.getZ(), tValue);
	}

	////////////////////////////////////////////////////////////////////////////////
	/// Note that if the memory usage limit is not large enough to support the region this function will only load part of the region. In this case it is undefined which parts will actually be loaded. If all the voxels in the given region are already loaded, this function will not do anything. Other voxels might be unloaded to make space for the new voxels.
	/// \param regPrefetch The Region of voxels to prefetch into memory.
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::prefetch(const Region& regPrefetch)
	{
		applyOnChunks(regPrefetch,
			[](const std::shared_ptr<Chunk>& chunk) { /* Nothing to do, just touch the chunks */ },
			[this](int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) { return getChunk(uChunkX, uChunkY, uChunkZ); });
	}

	////////////////////////////////////////////////////////////////////////////////
	/// Queries the status of the region:
	/// \return Available if in memory, Locked if at least partially locked (load in progress for instance), NotAvailable if not all in memory
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	RegionStatus PagedVolume<VoxelType, RWLockType>::getRegionStatus(const Region& regQuery)
	{
		RegionStatus status = RegionStatus::Available;
		applyOnChunks(regQuery,
			[this, &status](const std::shared_ptr<Chunk>& chunk)
			{
				if (!chunk)
				{
					status = RegionStatus::NotAvailable;
				}
				else if (status != RegionStatus::NotAvailable)
				{
					if (chunk->isLocked())
					{
						status = RegionStatus::Locked;
					}
				}
			},
			[this](int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) { return tryGetChunk(uChunkX, uChunkY, uChunkZ); });
		return status;
	}

	////////////////////////////////////////////////////////////////////////////////
	/// Removes all the voxels in the given region from memory, and calls dataOverflowHandler() to ensure the application has a chance to store the data.
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::flush(const Region& regFlush)
	{
		AutoWriteLock<RWLockType> lock(&m_internalLock);

		applyOnChunks(regFlush,
			[this](std::tuple<int32_t, int32_t, int32_t> chunkPos)
			{
				int32_t uChunkX, uChunkY, uChunkZ;
				std::tie(uChunkX, uChunkY, uChunkZ) = chunkPos;

				const uint32_t iPosisionHash = getPositionHash(uChunkX, uChunkY, uChunkZ);

				// Starting at the position indicated by the hash, and then search through the whole array looking for a chunk with the correct
				// position. In most cases we expect to find it in the first place we look. Note that this algorithm is slow in the case that
				// the chunk is not found because the whole array has to be searched, but in this case we are going to have to page the data in
				// from an external source which is likely to be slow anyway.
				uint32_t iIndex = iPosisionHash;
				do
				{
					if (m_arrayChunks[iIndex])
					{
						Vector3DInt32& entryPos = m_arrayChunks[iIndex]->m_v3dChunkSpacePosition;
						if (entryPos.getX() == uChunkX && entryPos.getY() == uChunkY && entryPos.getZ() == uChunkZ)
						{
							m_arrayChunks[iIndex].reset();
							// NOTE(dynworld) P3b: keep the incremental chunk count in sync.
							m_uChunkCount--;
							break;
						}
					}

					iIndex++;
					iIndex %= uChunkArraySize;
				} while (iIndex != iPosisionHash); // Keep searching until we get back to our start position.
			},
			// Forward chunk position as a tuple
			[](int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ)
			{
				return std::tie(uChunkX, uChunkY, uChunkZ);
			}
		);
	}

	////////////////////////////////////////////////////////////////////////////////
	/// Removes all voxels from memory, and calls dataOverflowHandler() to ensure the application has a chance to store the data.
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::flushAll()
	{
		// Clear chunk cache as all chunks are about to be removed.
		m_pChunkCache->dirtyCache();

		AutoWriteLock<RWLockType> lock(&m_internalLock);

		// Erase all the most recently used chunks.
		for (uint32_t uIndex = 0; uIndex < uChunkArraySize; uIndex++)
		{
			m_arrayChunks[uIndex] = nullptr;
		}

		// NOTE(dynworld) P3b: keep the incremental chunk count in sync.
		m_uChunkCount = 0;
	}

	template <typename VoxelType, typename RWLockType>
	template <typename Get, typename F>
	void PagedVolume<VoxelType, RWLockType>::applyOnChunks(const Region& region, F operation, Get getChunkF) const
	{
		// Convert the start and end positions into chunk space coordinates
		Vector3DInt32 v3dStart;
		for (int i = 0; i < 3; i++)
		{
			v3dStart.setElement(i, region.getLowerCorner().getElement(i) >> m_uChunkSideLengthPower);
		}

		Vector3DInt32 v3dEnd;
		for (int i = 0; i < 3; i++)
		{
			v3dEnd.setElement(i, region.getUpperCorner().getElement(i) >> m_uChunkSideLengthPower);
		}

		// Ensure we don't page in more chunks than the volume can hold.
		Region chunkRegion(v3dStart, v3dEnd);
		uint32_t uNoOfChunks = static_cast<uint32_t>(chunkRegion.getWidthInVoxels() * chunkRegion.getHeightInVoxels() * chunkRegion.getDepthInVoxels());
		POLYVOX_LOG_WARNING_IF(uNoOfChunks > m_uChunkCountLimit, "Attempting to access more than the maximum number of chunks (this will cause thrashing).");
		uNoOfChunks = (std::min)(uNoOfChunks, m_uChunkCountLimit);

		// Loops over the specified positions and touch the corresponding chunks.
		for (int32_t x = v3dStart.getX(); x <= v3dEnd.getX(); x++)
		{
			for (int32_t y = v3dStart.getY(); y <= v3dEnd.getY(); y++)
			{
				for (int32_t z = v3dStart.getZ(); z <= v3dEnd.getZ(); z++)
				{
					operation(getChunkF(x, y, z));
				}
			}
		}
	}

	template <typename VoxelType, typename RWLockType>
	std::shared_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> PagedVolume<VoxelType, RWLockType>::getExistingChunk(uint32_t iPosisionHash, int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const
	{
		std::shared_ptr<Chunk> pChunk;

		// Starting at the position indicated by the hash, and then search through the whole array looking for a chunk with the correct
		// position. In most cases we expect to find it in the first place we look. Note that this algorithm is slow in the case that
		// the chunk is not found because the whole array has to be searched, but in this case we are going to have to page the data in
		// from an external source which is likely to be slow anyway.
		// NOTE(dynworld) P3a': this lookup no longer acquires the chunk read
		// lock (which used to block it until an in-progress pageIn released the
		// chunk write lock, while the caller held the internal lock read). The
		// wait now happens in getChunk on the chunk's paging flag, with no
		// locks held; callers must honor isPagingInProgress() before reading
		// the buffer (getVoxel/setVoxel/the samplers do).
		uint32_t iIndex = iPosisionHash;
		do
		{
			if (m_arrayChunks[iIndex])
			{
				Vector3DInt32& entryPos = m_arrayChunks[iIndex]->m_v3dChunkSpacePosition;
				if (entryPos.getX() == uChunkX && entryPos.getY() == uChunkY && entryPos.getZ() == uChunkZ)
				{
					pChunk = m_arrayChunks[iIndex];
					pChunk->m_uChunkLastAccessed = RWLockType::preIncr(m_uTimestamper);
					break;
				}
			}

			iIndex++;
			iIndex %= uChunkArraySize;
		} while (iIndex != iPosisionHash); // Keep searching until we get back to our start position.

		return pChunk;
	}

	template <typename VoxelType, typename RWLockType>
	uint32_t PagedVolume<VoxelType, RWLockType>::getPositionHash(int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const
	{
		// We generate a 16-bit hash here and assume this matches the range available in the chunk
		// array. The assert here is just to make sure we take care if change this in the future.
		static_assert(uChunkArraySize == 65536, "Chunk array size has changed, check if the hash calculation needs updating.");
		// Extract the lower five bits from each position component.
		const uint32_t uChunkXLowerBits = static_cast<uint32_t>(uChunkX & 0x1F);
		const uint32_t uChunkYLowerBits = static_cast<uint32_t>(uChunkY & 0x1F);
		const uint32_t uChunkZLowerBits = static_cast<uint32_t>(uChunkZ & 0x1F);
		// Combine then to form a 15-bit hash of the position. Also shift by one to spread the values out in the whole 16-bit space.
		const uint32_t iPositionHash = (((uChunkXLowerBits)) | ((uChunkYLowerBits) << 5) | ((uChunkZLowerBits) << 10) << 1);

		return iPositionHash;
	}

	template <typename VoxelType, typename RWLockType>
	std::shared_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> PagedVolume<VoxelType, RWLockType>::tryGetChunk(int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const
	{
		std::shared_ptr<Chunk> pChunk;

		const uint32_t iPosisionHash = getPositionHash(uChunkX, uChunkY, uChunkZ);

		{ // Try to get the chunk in read-only mode
			AutoReadLock<RWLockType> lock(&m_internalLock);
			pChunk = getExistingChunk(iPosisionHash, uChunkX, uChunkY, uChunkZ);
		}

		return pChunk;
	}

	template <typename VoxelType, typename RWLockType>
	std::shared_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> PagedVolume<VoxelType, RWLockType>::getChunk(int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const
	{
		std::shared_ptr<Chunk> pChunk;

		const uint32_t iPosisionHash = getPositionHash(uChunkX, uChunkY, uChunkZ);

		// NOTE(dynworld) P3a': the loop below re-runs the lookup after waiting
		// on a chunk whose page-in is in progress, so that a chunk which got
		// evicted (or replaced by another creator) during the wait is not
		// returned stale.
		while (true)
		{
			{ // First try to get the chunk in read-only mode
				// NOTE(dynworld): timed as the reader-wait site (VPL) - this lookup no
				// longer blocks on the chunk read lock while an in-progress pageIn
				// fills the buffer; that wait moved to the flag spin below (P3a'),
				// so the volume internal lock read is not held across the wait (P3c).
				PIXBeginEvent(PIX_COLOR_DEFAULT, "ReadLookupChunk");
				AutoReadLock<RWLockType> lock(&m_internalLock);
				pChunk = getExistingChunk(iPosisionHash, uChunkX, uChunkY, uChunkZ);
				PIXEndEvent();
			}

			if (pChunk)
			{
				if (!pChunk->isPagingInProgress())
					break; // chunk is ready

				// Pager work in progress: wait WITHOUT holding any volume lock
				// (the pager body runs with no chunk write lock anymore), then
				// re-lookup in case the chunk was evicted/recreated meanwhile.
				// NOTE(dynworld): timed as the page-in wait site.
				PIXBeginEvent(PIX_COLOR_DEFAULT, "WaitPagingChunk");
				while (pChunk->isPagingInProgress())
					std::this_thread::yield();
				PIXEndEvent();
				continue;
			}

			bool pageInChunk = false;
			{
				// If the chunk was not found, attempt to get it with exclusive lock in case one created it in the meantime.
				AutoWriteLock<RWLockType> writeLock(&m_internalLock);
				PIXBeginEvent(PIX_COLOR_DEFAULT, "GetExistingChunk");
				pChunk = getExistingChunk(iPosisionHash, uChunkX, uChunkY, uChunkZ);
				PIXEndEvent();

				// If we still haven't found the chunk then it's time to create one.
				if (!pChunk)
				{
					// The chunk was not found so we will create a new one.
					PIXBeginEvent(PIX_COLOR_DEFAULT, "CreateChunk");
					Vector3DInt32 v3dChunkPos(uChunkX, uChunkY, uChunkZ);
					pChunk = std::make_shared<Chunk>(v3dChunkPos, m_uChunkSideLength, m_pPager);
					// NOTE(dynworld) P3a': the flag is raised BEFORE the chunk is
					// published into the array so no reader can observe the
					// uninitialized buffer; the chunk write lock is no longer
					// taken for the page-in window - readers wait on the flag
					// with no locks held instead.
					pChunk->m_bPagingInProgress.store(true, std::memory_order_seq_cst);
					pageInChunk = true;
					PIXEndEvent();

					pChunk->m_uChunkLastAccessed = RWLockType::preIncr(m_uTimestamper); // Important, as we may soon delete the oldest chunk

					// Store the chunk at the appropriate place in out chunk array. Ideally this place is
					// given by the hash, otherwise we do a linear search for the next available location
					// We always expect to find a free place because we aim to keep the array only half full.
					uint32_t iIndex = iPosisionHash;
					bool bInsertedSucessfully = false;
					PIXBeginEvent(PIX_COLOR_DEFAULT, "SearchFreeChunkSlot");
					do
					{
						if (m_arrayChunks[iIndex] == nullptr)
						{
							m_arrayChunks[iIndex] = pChunk;
							bInsertedSucessfully = true;
							break;
						}

						iIndex++;
						iIndex %= uChunkArraySize;
					} while (iIndex != iPosisionHash); // Keep searching until we get back to our start position.
					PIXEndEvent();

					// This should never really happen unless we are failing to keep our number of active chunks
					// significantly under the target amount. Perhaps if chunks are 'pinned' for threading purposes?
					POLYVOX_THROW_IF(!bInsertedSucessfully, std::logic_error, "No space in chunk array for new chunk.");

					// NOTE(dynworld) P3b: the chunk count is maintained incrementally,
					// so the O(uChunkArraySize) oldest-chunk scan only runs when the
					// limit is actually exceeded (not on every creation).
					m_uChunkCount++;

					// As we have added a chunk we may have exceeded our target chunk limit. Search through the array to
					// find the oldest timestamp. Paging the data in is probably more expensive than this scan, but it
					// is kept out of the common path now.
					if (m_uChunkCount > m_uChunkCountLimit)
					{
						uint32_t uOldestChunkIndex = 0;
						uint32_t uOldestChunkTimestamp = std::numeric_limits<uint32_t>::max();
						PIXBeginEvent(PIX_COLOR_DEFAULT, "SearchOldestChunk");
						for (uint32_t uIndex = 0; uIndex < uChunkArraySize; uIndex++)
						{
							if (m_arrayChunks[uIndex])
							{
								// NOTE(dynworld) P3a': never pick a chunk whose pageIn is
								// still in progress as an eviction candidate - its buffer
								// is being filled outside all locks by the pager thread.
								if (m_arrayChunks[uIndex]->m_bPagingInProgress.load(std::memory_order_relaxed))
									continue;

								if (m_arrayChunks[uIndex]->m_uChunkLastAccessed < uOldestChunkTimestamp)
								{
									uOldestChunkTimestamp = m_arrayChunks[uIndex]->m_uChunkLastAccessed;
									uOldestChunkIndex = uIndex;
								}
							}
						}
						PIXEndEvent();

						// Check if we have too many chunks, and delete the oldest if so.
						if (uOldestChunkTimestamp != std::numeric_limits<uint32_t>::max())
						{
							PIXScopedEvent(PIX_COLOR_DEFAULT, "DeleteOldestChunk");
							m_arrayChunks[uOldestChunkIndex] = nullptr;
							m_uChunkCount--;
						}
					}
				}
			}
			if (pageInChunk)
			{
				PIXScopedEvent(PIX_COLOR_DEFAULT, "PageInChunk");
				pChunk->pageIn();

				// NOTE(dynworld) P3a' finalize: the chunk lock was never taken for
				// the page-in window; this brief lock pair is a hard barrier that
				// serializes the pager's buffer writes against any chunk-lock user
				// (e.g. a region lock) that raced past the flag check, before the
				// flag release-store below publishes the completed data.
				pChunk->writeLock();
				pChunk->writeUnlock();
				pChunk->m_bPagingInProgress.store(false, std::memory_order_seq_cst);
				break;
			}

			// The chunk was created by another thread and may still be paging in;
			// loop back through the read branch so the paging-flag wait applies
			// to us as well.
		}

		m_pChunkCache->setChunkCached(this, pChunk, uChunkX, uChunkY, uChunkZ);

		return pChunk;
	}

	////////////////////////////////////////////////////////////////////////////////
	/// Calculate the memory usage of the volume.
	////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType>
	uint32_t PagedVolume<VoxelType, RWLockType>::calculateSizeInBytes(void)
	{
		AutoReadLock<RWLockType> lock(&m_internalLock);

		// NOTE(dynworld) P3b: the chunk count is maintained incrementally, no
		// O(uChunkArraySize) scan needed here anymore.

		// Note: We disregard the size of the other class members as they are likely to be very small compared to the size of the
		// allocated voxel data. This also keeps the reported size as a power of two, which makes other memory calculations easier.
		return _Myt::Chunk::calculateSizeInBytes(m_uChunkSideLength) * m_uChunkCount;
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::readLock() const
	{
		m_volumeLock.readLock();
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::readUnlock() const
	{
		m_volumeLock.readUnlock();
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::writeLock() const
	{
		m_volumeLock.writeLock();
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::writeUnlock() const
	{
		m_volumeLock.writeUnlock();
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::upgradeLock() const
	{
		m_volumeLock.upgradeLock();
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::downgradeLock() const
	{
		m_volumeLock.downgradeLock();
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::readLockRegion(const Region& region) const
	{
		applyOnChunks(region,
			[this](std::shared_ptr<Chunk> chunk) { chunk->readLock(); },
			[this](int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) { return getChunk(uChunkX, uChunkY, uChunkZ); });
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::readUnlockRegion(const Region& region) const
	{
		applyOnChunks(region,
			[this](std::shared_ptr<Chunk> chunk) { chunk->readUnlock(); },
			[this](int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) { return getChunk(uChunkX, uChunkY, uChunkZ); });
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::writeLockRegion(const Region& region) const
	{
		applyOnChunks(region,
			[this](std::shared_ptr<Chunk> chunk) { chunk->writeLock(); },
			[this](int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) { return getChunk(uChunkX, uChunkY, uChunkZ); });
	}

	template <typename VoxelType, typename RWLockType>
	void PagedVolume<VoxelType, RWLockType>::writeUnlockRegion(const Region& region) const
	{
		applyOnChunks(region,
			[this](std::shared_ptr<Chunk> chunk) { chunk->writeUnlock(); },
			[this](int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) { return getChunk(uChunkX, uChunkY, uChunkZ); });
	}

	template <typename VoxelType, typename RWLockType>
	PagedVolume<VoxelType, RWLockType>::AutoReadLockRegion::AutoReadLockRegion(const _Myt* volume, const Region& region)
		: m_volume(volume), m_region(region)
	{
		m_volume->readLockRegion(m_region);
	}

	template <typename VoxelType, typename RWLockType>
	PagedVolume<VoxelType, RWLockType>::AutoReadLockRegion::~AutoReadLockRegion()
	{
		m_volume->readUnlockRegion(m_region);
	}

	template <typename VoxelType, typename RWLockType>
	PagedVolume<VoxelType, RWLockType>::AutoWriteLockRegion::AutoWriteLockRegion(const _Myt* volume, const Region& region)
		: m_volume(volume), m_region(region)
	{
		m_volume->writeLockRegion(m_region);
	}

	template <typename VoxelType, typename RWLockType>
	PagedVolume<VoxelType, RWLockType>::AutoWriteLockRegion::~AutoWriteLockRegion()
	{
		m_volume->writeUnlockRegion(m_region);
	}
}

