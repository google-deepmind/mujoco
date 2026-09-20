/**
 * This script hides Sphinx auto-generated attribute links from the
 * "On this page" sidebar.
 * Adapted for the Furo theme.
 * Uses a MutationObserver to wait for Furo's JavaScript to create the
 * sidebar before attempting to modify it.
 */
function hideAttributesFromTOC() {
  // Furo's "On this page" sidebar container has the class .toc-tree
  const tocContainerSelector = '.toc-tree';

  // Step 1: Find all attribute definitions and collect their IDs.
  const attributeDefs = document.querySelectorAll('dl.py.attribute');
  if (attributeDefs.length === 0) {
    return false;  // No attributes on this page, nothing to do.
  }
  const attributeIds = new Set();
  attributeDefs.forEach(def => {
    const term = def.querySelector('dt');
    if (term && term.id) {
      attributeIds.add(term.id);
    }
  });

  if (attributeIds.size === 0) {
    return false;  // No attribute IDs found.
  }

  // Step 2: Find the sidebar container.
  const tocContainer = document.querySelector(tocContainerSelector);
  if (!tocContainer) {
    // Container not found yet. The observer will try again.
    return false;
  }

  // Step 3: Get all links within that sidebar.
  const tocLinks = tocContainer.querySelectorAll('a');
  if (tocLinks.length === 0) {
    return false;  // Container found, but it's empty. Let's wait.
  }

  // Step 4: Iterate through the links and hide the ones that match.
  let hiddenCount = 0;
  tocLinks.forEach(link => {
    const href = link.getAttribute('href');
    if (href && href.startsWith('#')) {
      const linkId = href.substring(1);
      if (attributeIds.has(linkId)) {
        // In Furo, the link is inside a list item (<li>) which we need to hide.
        const listItem = link.closest('li');
        if (listItem && listItem.style.display !== 'none') {
          console.log(`Hiding sidebar link for: #${linkId}`);
          listItem.style.display = 'none';
          hiddenCount++;
        }
      }
    }
  });

  // If we successfully hid the links, we can stop observing.
  if (hiddenCount > 0) {
    return true;  // Signal success
  }

  // If we found the container but didn't hide anything, maybe it's not fully
  // rendered. Let the observer run a few more times. A better approach might be
  // needed if this fails, but for most cases, this is sufficient.
  return false;
}

// Use a MutationObserver to wait for the page to be dynamically built.
const observer = new MutationObserver((mutations, obs) => {
  // We only need to run our function once successfully.
  if (hideAttributesFromTOC()) {
    obs.disconnect();  // Stop observing once the task is done.
  }
});

// Start observing the entire document body for added/removed nodes.
observer.observe(document.body, {childList: true, subtree: true});
